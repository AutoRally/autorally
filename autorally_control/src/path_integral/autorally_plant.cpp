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
 * @file autorally_plant.cpp
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Implementation of the AutorallyPlant class
 ***********************************************/
#include <autorally_control/path_integral/autorally_plant.h>

#include <stdio.h>
#include <stdlib.h>

namespace autorally_control {

AutorallyPlant::AutorallyPlant(ros::NodeHandle mppi_node, bool debug_mode, int hz)
{
  std::string pose_estimate_name;
  mppi_node.getParam("pose_estimate", pose_estimate_name);
  mppi_node.getParam("debug_mode", debug_mode_);
  //Initialize the publishers.
  control_pub_ = mppi_node.advertise<autorally_msgs::chassisCommand>("chassisCommand", 1);
  std::string nominal_path_name = "planned_trajectory";
  if (debug_mode_){
    nominal_path_name = "planned_trajectory_debug";
  }
  default_path_pub_ = mppi_node.advertise<nav_msgs::Path>(nominal_path_name, 1);
  delay_pub_ = mppi_node.advertise<geometry_msgs::Point>("mppiTimeDelay", 1);
  status_pub_ = mppi_node.advertise<autorally_msgs::pathIntegralStatus>("mppiStatus", 1);
  //Initialize the pose subscriber.
  pose_sub_ = mppi_node.subscribe(pose_estimate_name, 1, &AutorallyPlant::poseCall, this);
  //Initialize the servo subscriber
  servo_sub_ = mppi_node.subscribe("chassisState", 1, &AutorallyPlant::servoCall, this);
  //Initialize auxiliary variables.
  safe_speed_zero_ = false;
  debug_mode_ = debug_mode;
  activated_ = false;
  new_model_available_ = false;
  //Initialize timestamps to the current time.
  last_pose_call_ = ros::Time::now();
  last_check_ = ros::Time::now();
  //Initialize yaw derivative to zero
  full_state_.yaw_mder = 0.0;
  status_ = 1;
  if (debug_mode_){
    ocs_msg_ = "Debug Mode";
  }
  else {
    ocs_msg_ = "";
  }
  std::string info = "MPPI Controller";
  std::string hardwareID = "";
  std::string portPath = "";
  //Diagnostics::init(info, hardwareID, portPath);
}

AutorallyPlant::~AutorallyPlant(){}

void AutorallyPlant::poseCall(nav_msgs::Odometry pose_msg)
{
  //Update the timestamp
  last_pose_call_ = ros::Time::now();
  //Set activated to true --> we are receiving state messages.
  activated_ = true;
  //Update position
  full_state_.x_pos = pose_msg.pose.pose.position.x;
  full_state_.y_pos = pose_msg.pose.pose.position.y;
  full_state_.z_pos = pose_msg.pose.pose.position.z;
  //Grab the quaternion
  float q0 = pose_msg.pose.pose.orientation.w;
  float q1 = pose_msg.pose.pose.orientation.x;
  float q2 = pose_msg.pose.pose.orientation.y;
  float q3 = pose_msg.pose.pose.orientation.z;
  	//Update euler angles. These use the 1-2-3 Euler angle convention.
  full_state_.roll = atan2(2*q2*q3 + 2*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0);
  full_state_.pitch = -asin(2*q1*q3 - 2*q0*q2);
  full_state_.yaw = atan2(2*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);
  //Update the quaternion
  full_state_.q0 = q0;
  full_state_.q1 = q1;
  full_state_.q2 = q2;
  full_state_.q3 = q3;
  //Update the world frame velocity
  full_state_.x_vel = pose_msg.twist.twist.linear.x;
  full_state_.y_vel = pose_msg.twist.twist.linear.y;
  full_state_.z_vel = pose_msg.twist.twist.linear.z;
  //Update the body frame longitudenal and lateral velocity
  full_state_.u_x = cos(full_state_.yaw)*full_state_.x_vel + sin(full_state_.yaw)*full_state_.y_vel;
  full_state_.u_y = -sin(full_state_.yaw)*full_state_.x_vel + cos(full_state_.yaw)*full_state_.y_vel;
  //Update the minus yaw derivative.
  full_state_.yaw_mder = -pose_msg.twist.twist.angular.z;//.5*full_state_.yaw_mder + .5*(-1.0/cos(full_state_.pitch)*(sin(full_state_.roll)*pose_msg.twist.twist.angular.y
  						            //  + cos(full_state_.roll)*pose_msg.twist.twist.angular.z));
}

void AutorallyPlant::servoCall(autorally_msgs::chassisState servo_msg)
{
  full_state_.steering = servo_msg.steering;
  full_state_.throttle = servo_msg.throttle;
}

void AutorallyPlant::runstopCall(autorally_msgs::runstop safe_msg)
{
  if (safe_msg.motionEnabled == false){
    safe_speed_zero_ = true;
  }
}

void AutorallyPlant::pubPath(float* nominal_traj, int num_timesteps, int hz)
{
  pubPath(nominal_traj, default_path_pub_, num_timesteps, hz);
}

void AutorallyPlant::pubPath(float* nominal_traj, ros::Publisher path_pub_, int num_timesteps, int hz)
{
  path_msg_.poses.clear();
  int i;
  float r,p,y,q0,q1,q2,q3;
  ros::Time begin = ros::Time::now();
  for (i = 0; i < num_timesteps; i++) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = nominal_traj[i*(AUTORALLY_STATE_DIM)];
    pose.pose.position.y = nominal_traj[i*(AUTORALLY_STATE_DIM) + 1];
    pose.pose.position.z = 0;
    y = nominal_traj[i*(AUTORALLY_STATE_DIM) + 2];
    r = nominal_traj[i*(AUTORALLY_STATE_DIM) + 3];
    p = 0;
    q0 = cos(r/2)*cos(p/2)*cos(y/2) - sin(r/2)*sin(p/2)*sin(y/2);
    q1 = cos(r/2)*cos(p/2)*sin(y/2) + sin(r/2)*cos(y/2)*sin(p/2);
    q2 = cos(r/2)*cos(y/2)*sin(p/2) - sin(r/2)*cos(p/2)*sin(y/2);
    q3 = cos(r/2)*sin(p/2)*sin(y/2) + cos(p/2)*cos(y/2)*sin(r/2);
    pose.pose.orientation.w = q0;
    pose.pose.orientation.x = q1;
    pose.pose.orientation.y = q2;
    pose.pose.orientation.z = q3;
    pose.header.stamp = begin;
    pose.header.frame_id = "odom";
    path_msg_.poses.push_back(pose);
  }
  path_msg_.header.stamp = begin;
  path_msg_.header.frame_id = "odom";
  path_pub_.publish(path_msg_);
}

void AutorallyPlant::pubControl(float steering, float throttle)
{
  //Publish the steering and throttle commands
  if (std::isnan(throttle) || std::isnan(steering)){ //Nan control publish zeros and exit.
    ROS_INFO("NaN Control Input Detected");
    control_msg_.steering = 0;
    control_msg_.throttle = -.99;
    control_msg_.frontBrake = -5.0;
    control_msg_.header.stamp = ros::Time::now();
    control_msg_.sender = "mppi_controller";
    control_pub_.publish(control_msg_);
    ros::shutdown(); //No use trying to recover, quitting is the best option.
  }
  else if (!activated_ && !debug_mode_) { //No state update received yet.
    control_msg_.steering = 0;
    control_msg_.throttle = 0;
    control_msg_.frontBrake = -5.0;
    control_msg_.header.stamp = ros::Time::now();
    control_msg_.sender = "mppi_controller";
    control_pub_.publish(control_msg_);
  }
  else { //Publish the computed control input.
    control_msg_.steering = steering;
    control_msg_.throttle = throttle;
    control_msg_.frontBrake = -5.0;
    control_msg_.header.stamp = ros::Time::now();
    control_msg_.sender = "mppi_controller";
    control_pub_.publish(control_msg_);
  }
}

void AutorallyPlant::pubStatus(){
  status_msg_.info = ocs_msg_;
  status_msg_.status = status_;
  status_msg_.header.stamp = ros::Time::now();
  status_pub_.publish(status_msg_);
}

AutorallyPlant::FullState AutorallyPlant::getState()
{
  return full_state_;
}

//Returns true is no pose estimate received in the last half second.
bool AutorallyPlant::getStale()
{
  double now = ros::Time::now().toSec();
  double last_pose = last_pose_call_.toSec();
  bool stale = false;
  if (now - last_pose > TIMEOUT) {
    stale = true;
  }
  return stale;
}

bool AutorallyPlant::getRunstop()
{
  return safe_speed_zero_;
}

ros::Time AutorallyPlant::getLastPoseTime()
{
  return last_pose_call_;
}

int AutorallyPlant::checkStatus()
{
  if (getStale() && activated_){ //Stale pose estimate.
    status_ = 2;
    ocs_msg_ = "POSE STALE";
  }
  else if (!activated_) {
    status_ = 1;
    ocs_msg_ = "No pose estimates received.";
  }
  else if (safe_speed_zero_){
    status_ = 1;
    ocs_msg_ = "Safe speed zero.";
  }
  else {
    ocs_msg_ = "Controller OK";
    status_ = 0; //Everything is good.
  }
  last_check_ = ros::Time::now(); //Set the last_check_ time variable to now.
  return status_;
}

} //namespace autorally_control
