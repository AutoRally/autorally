/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file gpsWaypoint.cpp
 * @author Paul Drews <pdrews3@gatech.edu>
 * @date January 29, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief ROS node to allow following gps waypoints
 *
 * @details Waypoints are read in from a configuration file.
 * The waypoints are assumed to be a loop.  Once each waypoint is reached,
 * the controller will push it to the back of the list and move to the next.
 ***********************************************/

#include <boost/algorithm/string.hpp>
#include <vector>
#include "gpsWaypoint.h"

namespace autorally_control
{

  GpsWaypoint::GpsWaypoint() :
    m_nh("~"), m_speed(0.0), m_useThetaGPS(true), m_prevTime(0.0)
  {
    m_nh.param("WaypointFile", m_filename, std::string("waypoints.txt"));
    m_nh.param("WaypointRadius", m_wpRadius, 1.5);
    m_nh.param("HeadingP", m_headingP, 2.0);
    
    m_speedSub = m_nh.subscribe("Speeds", 1, &GpsWaypoint::Speedcb, this);
    m_odomSub = m_nh.subscribe("Odom", 1, &GpsWaypoint::Odomcb, this);
//    m_poseSub = m_nh.subscribe("Pose", 1, &GpsWaypoint::Posecb, this);

    m_servoPub = m_nh.advertise<autorally_msgs::servoMSG>("WPController/servoCommand", 1);
    vis_pub = m_nh.advertise<visualization_msgs::Marker>( "markers", 200 );
    m_maskPub = m_nh.advertise<autorally_msgs::imageMask>("imageMask", 1);
    m_imMask.sender = "Waypoint Follower";
    m_imMask.lines.push_back(autorally_msgs::line2D());
    m_imMask.lines[0].start.x = 256;
    m_imMask.lines[0].start.y = 320;

    std::ifstream wptFile;
    ROS_INFO("Opening file %s", m_filename.c_str());
    wptFile.open(m_filename.c_str());
    std::string curLine;
//    m_paramTimer = m_nh.createTimer(ros::Rate(1),
//                   &GpsWaypoint::paramCallback, this);
    while(getline(wptFile, curLine))
    {
      double x, y;
      geometry_msgs::Point pt;
      std::vector<std::string> strs;
      boost::split(strs, curLine, boost::is_any_of(","));
      if (strs.size() == 2)
      pt.x = boost::lexical_cast<double>(strs[0]);
      pt.y = boost::lexical_cast<double>(strs[1]);
      ROS_INFO("Loaded waypoint %f %f", pt.x, pt.y);
      m_wpts.push_back(pt);
    }
    wptFile.close();
    m_offsetX = m_wpts.front().x;
    m_offsetY = m_wpts.front().y;

    dynamic_reconfigure::Server<gpsWaypoint_paramsConfig>::CallbackType cb;

    cb = boost::bind(&GpsWaypoint::ConfigCallback, this, _1, _2);
    m_dynServer.setCallback(cb);
    ros::Duration d = ros::Duration(1.0);
    d.sleep();
    PublishMarkers_track();
  }

  GpsWaypoint::~GpsWaypoint()
  {}
  
  void GpsWaypoint::Speedcb(autorally_msgs::wheelSpeeds speeds)
  {
    m_lock.lock();
    m_speed = (speeds.lfSpeed + speeds.rfSpeed) / 2.0;
    m_lock.unlock();
  }

  void GpsWaypoint::Odomcb(nav_msgs::Odometry position)
  {
    double speed;
    double x, y, theta;
    m_lock.lock();
    m_prevPos = m_position;
    m_position = position;
    speed = m_speed;
    x = position.pose.pose.position.x;
    y = position.pose.pose.position.y;

    // Get our heading from the message
    double roll,pitch,yaw;
    tf::Quaternion quat(position.pose.pose.orientation.x,
                      position.pose.pose.orientation.y,
                      position.pose.pose.orientation.z,
                      position.pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    double deltaX = x - m_prevPos.pose.pose.position.x;
    double deltaY = y - m_prevPos.pose.pose.position.y;
    double thetaGPS = atan2(deltaY,deltaX);
    m_lock.unlock();

    if (m_useThetaGPS)
      theta = thetaGPS;
    else
      theta = yaw;
    // Now we have all the latest data, do some maths
    // Are we at the next waypoint?
    double xn = m_wpts.front().x;
    double yn = m_wpts.front().y;
    while (GetDist(x,y,xn,yn) < m_wpRadius)
    {
      // Get the next wp
      m_wpts.push_back(m_wpts.front());
      m_wpts.pop_front();
      xn = m_wpts.front().x;
      yn = m_wpts.front().y;
      //ROS_INFO("Hit a waypoint");
    }
    // UTM is a ENU system
    // imugps node uses a local ENU system
    // so our heading is measured from the positive x axis, meaning 0 is pointing east
    // and this lets a normal x-y axis system drawn on paper represent our system with x pointing up.

    double bearing = atan2(yn-y,xn-x);
    double error = AngleDiff(bearing, theta);

    autorally_msgs::servoMSG servos;
    servos.throttle = -5.0;
    servos.frontBrake = -5.0;
    servos.backBrake = -5.0;
    servos.steering = Clamp(m_headingP * error,-1.0,1.0);

    servos.header.stamp = ros::Time::now();
    servos.header.frame_id = "WPController";
    m_servoPub.publish(servos);

    //m_imMask.lines[0].end.x = 256 - (100 * cos((servos.steering * PI / 2.0) + PI/2.0));
    //m_imMask.lines[0].end.y = 320 - (100 * sin((servos.steering * PI / 2.0) + PI/2.0));
    double time = ros::Time::now().toSec();

    if (time > (m_prevTime + 0.1))
    {
      m_prevTime = time;
      m_imMask.lines[0].end.x = 256 - (100 * sin(servos.steering * PI / 2.0));
      m_imMask.lines[0].end.y = 320 - (100 * cos(servos.steering * PI / 2.0));
      std::cout << " Error " << error << " Steering " << servos.steering;
      std::cout << "At theta " << theta << " bearing " << bearing;
      std::cout << " x " << x << " y " << y << std::endl;
      std::cout << " xn " << xn << " yn " << yn << std::endl;
      std::cout << " dx " << deltaX<< " dy " << deltaY << std::endl;
      m_maskPub.publish(m_imMask);

      PublishMarkers(x,y,theta,xn,yn);
    }

  }
//  void GpsWaypoint::Posecb(sensor_msgs::Imu pose)
//  {
//    m_lock.lock();
//    m_pose = pose;
//    m_lock.unlock();
//  }
//void GpsWaypoint::paramCallback(const ros::TimerEvent& time)
//{
//  m_nh.param("HeadingP", m_headingP, 2.0);
//}

  double GpsWaypoint::GetDist(double x1, double y1, double x2, double y2)
  {
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
  }

  double GpsWaypoint::AngleDiff(double a, double b)
  {
    double dif = fmod(a-b + PI, 2.0*PI);
    if (dif < 0)
      dif += 2.0*PI;
    return dif - PI;
  }

  double GpsWaypoint::Clamp(double num, double min, double max)
  {
    if (num < min)
      num = min;
    if (num > max)
      num = max;
    return num;
  }

  void GpsWaypoint::PublishMarkers(double x, double y, double yaw, double xwp, double ywp)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "LocalFrame";
    marker.header.stamp = ros::Time();
    marker.ns = "gps";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x - m_offsetX;
    marker.pose.position.y = y - m_offsetY;
    marker.pose.position.z = 1;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, (yaw ));
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    vis_pub.publish( marker );

    std::list<geometry_msgs::Point>::const_iterator iter;
    iter = m_wpts.begin();
    marker.header.frame_id = "LocalFrame";
    marker.header.stamp = ros::Time();
    marker.ns = "gps";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = iter->x - m_offsetX;
    marker.pose.position.y = iter->y - m_offsetY;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.7;
    marker.scale.y = 0.7;
    marker.scale.z = 0.7;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub.publish( marker );
}
  void GpsWaypoint::PublishMarkers_track()
  {
    visualization_msgs::Marker marker;

    marker.id = 2;
    std::list<geometry_msgs::Point>::const_iterator iter;
    for (iter = m_wpts.begin(); iter != m_wpts.end(); ++iter) {
      marker.header.frame_id = "LocalFrame";
      marker.header.stamp = ros::Time();
      marker.ns = "gps";
      marker.id ++;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = iter->x - m_offsetX;
      marker.pose.position.y = iter->y - m_offsetY;
      marker.pose.position.z = 1;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1;
      marker.scale.x = 0.7;
      marker.scale.y = 0.7;
      marker.scale.z = 0.7;
      marker.color.a = 0.2;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      vis_pub.publish( marker );
      std::cout << "Published " << marker.id << std::endl;
    }

    //ROS_INFO("Publish markers at %f %f and %f %f", x, y, xwp, ywp);
  }

  void GpsWaypoint::ConfigCallback(const gpsWaypoint_paramsConfig &config, uint32_t level)
  {
    m_headingP = config.pGain;
    m_useThetaGPS = config.gpsHeading;
    m_wpRadius = config.wpRadius;
    std::cout << "Got a config!!" << std::endl;
  }
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "GpsWaypoint");
  //ros::NodeHandle n;
  autorally_control::GpsWaypoint wpt;
  ros::spin();
}
