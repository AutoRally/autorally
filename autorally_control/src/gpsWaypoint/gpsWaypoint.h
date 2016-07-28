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
 * @file JumpControl.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date November 13, 2013
 * @copyright 2012 Georgia Institute of Technology
 * @brief JumpControl class definition
 *
 ***********************************************/

#ifndef GPS_WAYPOINT_H_
#define GPS_WAYPOINT_H_

#include <list>
#include <iostream>
#include <fstream>

#include <boost/thread.hpp>          // Mutex
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <autorally_msgs/chassisCommand.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/imageMask.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <autorally_control/gpsWaypoint_paramsConfig.h>

#define PI 3.14159265358979323846264338

namespace autorally_control
{
  class GpsWaypoint
  {
  public:
    GpsWaypoint();
    ~GpsWaypoint();
  private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_speedSub;
    ros::Subscriber m_odomSub;
    ros::Subscriber m_poseSub;
    ros::Publisher  m_chassisCommandPub;
    ros::Publisher vis_pub;
    ros::Timer m_paramTimer;


    std::string m_filename;

    std::list<geometry_msgs::Point> m_wpts;
//    sensor_msgs::Imu m_pose;
    nav_msgs::Odometry m_position;
    nav_msgs::Odometry m_prevPos;
    double m_speed;
    double m_wpRadius;
    double m_headingP;
    bool m_useThetaGPS;
    double m_offsetX, m_offsetY;
    double m_prevTime;

    dynamic_reconfigure::Server<gpsWaypoint_paramsConfig> m_dynServer;

    tf::TransformListener m_tf;

    boost::mutex m_lock;

    autorally_msgs::imageMask m_imMask;
    ros::Publisher m_maskPub; ///< Publisher for steering angle commands

    void Speedcb(autorally_msgs::wheelSpeeds speeds);
    void Odomcb(nav_msgs::Odometry position);
    void Posecb(sensor_msgs::Imu pose);
    void paramCallback(const ros::TimerEvent& time);
    double GetDist(double x1, double y1, double x2, double y2);
    double AngleDiff(double a, double b);
    double Clamp(double num, double min, double max);
    void PublishMarkers(double x, double y, double yaw, double xwp, double ywp);
    void PublishMarkers_track();
    void ConfigCallback(const gpsWaypoint_paramsConfig &config, uint32_t level);


  };
};
#endif 
