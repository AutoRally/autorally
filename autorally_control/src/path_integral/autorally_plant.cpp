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


AutorallyPlant::AutorallyPlant(ros::NodeHandle global_node, ros::NodeHandle mppi_node, bool debug_mode, int hz)
{
  std::string pose_estimate_name;
  mppi_node.getParam("pose_estimate", pose_estimate_name);
  mppi_node.getParam("debug_mode", debug_mode_);
  mppi_node.getParam("num_timesteps", numTimesteps_);
  deltaT_ = 1.0/hz;

  controlSequence_.resize(AUTORALLY_CONTROL_DIM*numTimesteps_);
  stateSequence_.resize(AUTORALLY_STATE_DIM*numTimesteps_);

  //Initialize the publishers.
  control_pub_ = mppi_node.advertise<autorally_msgs::chassisCommand>("chassisCommand", 1);
  path_pub_ = mppi_node.advertise<nav_msgs::Path>("nominalPath", 1);
  subscribed_pose_pub_ = mppi_node.advertise<nav_msgs::Odometry>("subscribedPose", 1);
  status_pub_ = mppi_node.advertise<autorally_msgs::pathIntegralStatus>("mppiStatus", 1);

  //Initialize the subscribers.
  pose_sub_ = global_node.subscribe(pose_estimate_name, 1, &AutorallyPlant::poseCall, this,
                                  ros::TransportHints().tcpNoDelay());
  servo_sub_ = global_node.subscribe("chassisState", 1, &AutorallyPlant::servoCall, this);
 
  //Timer callback for path publisher
  pathTimer_ = mppi_node.createTimer(ros::Duration(0.033), &AutorallyPlant::pubPath, this);
  statusTimer_ = mppi_node.createTimer(ros::Duration(0.033), &AutorallyPlant::pubStatus, this);
  debugImgTimer_ = mppi_node.createTimer(ros::Duration(0.033), &AutorallyPlant::displayDebugImage, this);

  //Initialize auxiliary variables.
  safe_speed_zero_ = false;
  debug_mode_ = debug_mode;
  activated_ = false;
  new_model_available_ = false;
  last_pose_call_ = ros::Time::now();
  
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
}

void AutorallyPlant::setSolution(std::vector<float> traj, std::vector<float> controls, 
                                util::EigenAlignedVector<float, 2, 7> gains,
                                ros::Time ts, double loop_speed)
{
  boost::mutex::scoped_lock lock(access_guard_);
  optimizationLoopTime_ = loop_speed;
  solutionTs_ = ts;
  for (int t = 0; t < numTimesteps_; t++){
    for (int i = 0; i < AUTORALLY_STATE_DIM; i++){
      stateSequence_[AUTORALLY_STATE_DIM*t + i] = traj[AUTORALLY_STATE_DIM*t + i];
    }
    for (int i = 0; i < AUTORALLY_CONTROL_DIM; i++){
      controlSequence_[AUTORALLY_CONTROL_DIM*t + i] = controls[AUTORALLY_CONTROL_DIM*t + i];
    }
  }
  feedback_gains_ = gains;
  solutionReceived = true;
}

void AutorallyPlant::setDebugImage(cv::Mat img)
{
  boost::mutex::scoped_lock lock(access_guard_);
  receivedDebugImg_ = true;
  debugImg_ = img;
}

void AutorallyPlant::displayDebugImage(const ros::TimerEvent&)
{
  /*{
  boost::mutex::scoped_lock lock(access_guard_);
    if (receivedDebugImg_) {
      cv::namedWindow("debugImage", cv::WINDOW_AUTOSIZE);
      cv::imshow("debugImage", debugImg_);
    } 
  }
  if (receivedDebugImg_){
    cv::waitKey(1);
  }*/
}


void AutorallyPlant::poseCall(nav_msgs::Odometry pose_msg)
{
  boost::mutex::scoped_lock lock(access_guard_);
  //Update the timestamp
  last_pose_call_ = pose_msg.header.stamp;
  poseCount_++;
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
  full_state_.yaw_mder = -pose_msg.twist.twist.angular.z;

  //Interpolate and publish the current control
  double timeFromLastOpt = (last_pose_call_ - solutionTs_).toSec();

  if (solutionReceived && timeFromLastOpt > 0 && timeFromLastOpt < (numTimesteps_-1)*deltaT_){
    double steering_ff, throttle_ff, steering_fb, throttle_fb, steering, throttle;
    int lowerIdx = (int)(timeFromLastOpt/deltaT_);
    int upperIdx = lowerIdx + 1;
    double alpha = (timeFromLastOpt - lowerIdx*deltaT_)/deltaT_;
    steering_ff = (1 - alpha)*controlSequence_[2*lowerIdx] + alpha*controlSequence_[2*upperIdx];
    throttle_ff = (1 - alpha)*controlSequence_[2*lowerIdx + 1] + alpha*controlSequence_[2*upperIdx + 1];

    Eigen::MatrixXf current_state(7,1);
    Eigen::MatrixXf desired_state(7,1);
    Eigen::MatrixXf deltaU;
    current_state << full_state_.x_pos, full_state_.y_pos, full_state_.yaw, full_state_.roll, full_state_.u_x, full_state_.u_y, full_state_.yaw_mder;
    for (int i = 0; i < 7; i++){
      current_state(i) = (1 - alpha)*controlSequence_[7*lowerIdx + i] + alpha*controlSequence_[7*upperIdx + i];
    }
    
    deltaU = ((1-alpha)*feedback_gains_[lowerIdx] + alpha*feedback_gains_[upperIdx])*(current_state - desired_state);

    if (std::isnan( deltaU(0) ) || std::isnan( deltaU(1))){
      pubControl(steering_ff, throttle_ff);
    }
    else {
      steering_fb = deltaU(0);
      throttle_fb = deltaU(1);
      steering = fmin(0.99, fmax(-0.99, steering_ff + steering_fb));
      throttle = fmin(0.75, fmax(-0.99, throttle_ff + throttle_fb));
      pubControl(steering, throttle);
    }
  }
}

void AutorallyPlant::servoCall(autorally_msgs::chassisState servo_msg)
{
  boost::mutex::scoped_lock lock(access_guard_);
  full_state_.steering = servo_msg.steering;
  full_state_.throttle = servo_msg.throttle;
}

void AutorallyPlant::runstopCall(autorally_msgs::runstop safe_msg)
{
  boost::mutex::scoped_lock lock(access_guard_);
  if (safe_msg.motionEnabled == false){
    safe_speed_zero_ = true;
  }
}

void AutorallyPlant::pubPath(const ros::TimerEvent&)
{
  boost::mutex::scoped_lock lock(access_guard_);
  path_msg_.poses.clear();
  nav_msgs::Odometry subscribed_state;
  int i;
  float phi,theta,psi,q0,q1,q2,q3;
  ros::Time begin = solutionTs_;
  for (int i = 0; i < numTimesteps_; i++) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = stateSequence_[i*(AUTORALLY_STATE_DIM)];
    pose.pose.position.y = stateSequence_[i*(AUTORALLY_STATE_DIM) + 1];
    pose.pose.position.z = 0;
    psi = stateSequence_[i*(AUTORALLY_STATE_DIM) + 2];
    phi = stateSequence_[i*(AUTORALLY_STATE_DIM) + 3];
    theta = 0;
    q0 = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
    q1 = -cos(phi/2)*sin(theta/2)*sin(psi/2) + cos(theta/2)*cos(psi/2)*sin(phi/2);
    q2 = cos(phi/2)*cos(psi/2)*sin(theta/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
    q3 = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*cos(psi/2)*sin(theta/2); 
    pose.pose.orientation.w = q0;
    pose.pose.orientation.x = q1;
    pose.pose.orientation.y = q2;
    pose.pose.orientation.z = q3;
    pose.header.stamp = begin + ros::Duration(i*deltaT_);
    pose.header.frame_id = "odom";
    path_msg_.poses.push_back(pose);
    if (i == 0){
      subscribed_state.pose.pose = pose.pose;
      subscribed_state.twist.twist.linear.x = stateSequence_[4];
      subscribed_state.twist.twist.linear.y = stateSequence_[5];
      subscribed_state.twist.twist.angular.z = -stateSequence_[6];
    }
  }
  subscribed_state.header.stamp = begin;
  subscribed_state.header.frame_id = "odom";
  path_msg_.header.stamp = begin;
  path_msg_.header.frame_id = "odom";
  path_pub_.publish(path_msg_);
  subscribed_pose_pub_.publish(subscribed_state);
}

void AutorallyPlant::pubControl(float steering, float throttle)
{
  autorally_msgs::chassisCommand control_msg; ///< Autorally control message initialization.
  std::cout << steering << ", " << throttle << std::endl;
  //Publish the steering and throttle commands
  if (std::isnan(throttle) || std::isnan(steering)){ //Nan control publish zeros and exit.
    ROS_INFO("NaN Control Input Detected");
    control_msg.steering = 0;
    control_msg.throttle = -.99;
    control_msg.frontBrake = -5.0;
    control_msg.header.stamp = ros::Time::now();
    control_msg.sender = "mppi_controller";
    control_pub_.publish(control_msg);
    ros::shutdown(); //No use trying to recover, quitting is the best option.
  }
  else { //Publish the computed control input.
    control_msg.steering = steering;
    control_msg.throttle = throttle;
    control_msg.frontBrake = -5.0;
    control_msg.header.stamp = ros::Time::now();
    control_msg.sender = "mppi_controller";
    control_pub_.publish(control_msg);
  }
}

void AutorallyPlant::pubStatus(const ros::TimerEvent&){
  boost::mutex::scoped_lock lock(access_guard_);
  status_msg_.info = ocs_msg_;
  status_msg_.status = status_;
  status_msg_.header.stamp = ros::Time::now();
  status_pub_.publish(status_msg_);
}

AutorallyPlant::FullState AutorallyPlant::getState()
{
  boost::mutex::scoped_lock lock(access_guard_);
  return full_state_;
}

bool AutorallyPlant::getRunstop()
{
  boost::mutex::scoped_lock lock(access_guard_);
  return safe_speed_zero_;
}

ros::Time AutorallyPlant::getLastPoseTime()
{
  boost::mutex::scoped_lock lock(access_guard_);
  return last_pose_call_;
}

int AutorallyPlant::checkStatus()
{
  boost::mutex::scoped_lock lock(access_guard_);
  if (!activated_) {
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
  return status_;
}

} //namespace autorally_control
