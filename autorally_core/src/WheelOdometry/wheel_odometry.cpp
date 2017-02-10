/*
* Software License Agreement (BSD License)
* Copyright (c) 2016, Georgia Institute of Technology
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
/**
 * @file wheel_odometry.cpp
 * @author Justin Zheng <justinyzheng@gmail.com>
 * @date August 8, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief Odometry node
 *
 * @details Uses wheel speeds and servo commands to estimate linear velocities and yaw rate
 **/
#include "wheel_odometry.h"
#include <tf/transform_datatypes.h>

namespace autorally_core
{
WheelOdometry::WheelOdometry()
{
  ros::NodeHandle n;

  n.getParam("vehicle_wheelbase", length_);
  n.getParam("vehicle_width", width_);
  n.getParam("using_sim", using_sim_);
  n.getParam("time_delay", time_delay_);
  // debug mode publishes a different message and subscribes to state estimator for easy visualization
  n.getParam("debug", debug_);

  servo_sub_ = n.subscribe("chassisState", 1, &WheelOdometry::servoCallback, this);
  wheel_speeds_sub_ = n.subscribe("wheelSpeeds", 1, &WheelOdometry::speedCallback, this);
  if (debug_)
    state_estimator_sub_ = n.subscribe("pose_estimate", 1, &WheelOdometry::stateEstimatorCallback, this);

  // time_delay parameter is measured in seconds - only used in debug mode
  // must be transformed to a number of messages (from /wheelSpeeds) to delay calculations of angular velocity
  double num_delay = round(time_delay_ * 70.0); // /wheelSpeeds publish frequency is 70Hz
  angular_velocities_ = std::vector<double>(num_delay, 0.0);
  time_step_ = std::vector<double>(num_delay, 0.02); // initialize with 0.02s as time step

  odom_ = n.advertise<nav_msgs::Odometry>("wheel_odom", 1);
}

WheelOdometry::~WheelOdometry()
{
}

void WheelOdometry::stateEstimatorCallback(const nav_msgs::OdometryConstPtr& state_estimator_msg)
{
  double w = state_estimator_msg->pose.pose.orientation.w;
  double x = state_estimator_msg->pose.pose.orientation.x;
  double y = state_estimator_msg->pose.pose.orientation.y;
  double z = state_estimator_msg->pose.pose.orientation.z;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, yaw_angle_);

  // the /pose_estime does not conform to usual ROS standard of twist in local frame
  // X velocity in local frame
  delta_x_state_estimator_ = cos(yaw_angle_) * state_estimator_msg->twist.twist.linear.x + sin(yaw_angle_)
        * state_estimator_msg->twist.twist.linear.y;

  // Y velocity in local frame
  delta_y_state_estimator_ = -1 * sin(yaw_angle_) * state_estimator_msg->twist.twist.linear.x + cos(yaw_angle_)
        * state_estimator_msg->twist.twist.linear.y;

  delta_theta_state_estimator_ = state_estimator_msg->twist.twist.angular.z;

  // Initialize x, y, and heading to match state estimator
  if (!initial_pose_received_)
  { 
    x_ = state_estimator_msg->pose.pose.position.x;
    y_ = state_estimator_msg->pose.pose.position.y;
    theta_ = yaw_angle_ * 180 / PI;
    initial_pose_received_ = true;
  }
}

void WheelOdometry::servoCallback(const autorally_msgs::chassisStateConstPtr& servo)
{
  // mapping servo values to their corresponding steering angle
  // Servo values are negative for left turns, steering angle is positive for left turns
  servo_val_ = servo->steering;
  if (!using_sim_)
  {
    // correct for values to high and too low
    if (servo_val_ >= MAX_SERVO_VAL)
      steering_angle_ = STEERING_ALPHA * MAX_SERVO_VAL + STEERING_BETA;
    else if (servo_val_ <= -MAX_SERVO_VAL)
      steering_angle_ = STEERING_ALPHA * -MAX_SERVO_VAL + STEERING_BETA;
    else
      steering_angle_ = STEERING_ALPHA * servo_val_ + STEERING_BETA;
  }
  else
  {
      //Simulator steering is ideal
      steering_angle_ = -21 * servo_val_;
  }
}

void WheelOdometry::speedCallback(const autorally_msgs::wheelSpeedsConstPtr& speed)
{
  // For first timestep, use .02 which is approximately the timestep between wheelSpeeds messages
  if (prev_ == 0)
    delta_t_ = .02;
  else
    delta_t_ = speed->header.stamp.toSec() - prev_;
  prev_ = speed->header.stamp.toSec();

  speed_FL_ = speed->lfSpeed;
  speed_FR_ = speed->rfSpeed;
  speed_BL_ = speed->lbSpeed;
  speed_BR_ = speed->rbSpeed;
  avg_speed_ = (speed_FL_ + speed_FR_) / 2;

  if (std::abs(steering_angle_) < 1e-6)
  {
    // approximately straight steering - turn radius +inf
    delta_x_ = avg_speed_ * delta_t_;
    delta_y_ = 0;
    delta_theta_ = 0;
  }
  else
  {
    turn_radius_ = length_ / sin(std::abs(steering_angle_) * PI / 180.0);
    double turning_phi = avg_speed_ * delta_t_ / turn_radius_;
    delta_x_ = turn_radius_ * sin(turning_phi);
    if (steering_angle_ > 0)
    {
      delta_y_ = turn_radius_ - turn_radius_ * cos(turning_phi);
    }
    else if (steering_angle_ < 0)
    {
      delta_y_ = -(turn_radius_ - turn_radius_ * cos(turning_phi));
    }
    delta_theta_ = avg_speed_ / length_ * sin(steering_angle_ * PI / 180.0) * 180 / PI * delta_t_;
  }


  // Delay this node's angular velocity estimate to account for system delays
  // useful for lining up state estimator values and odometry values for data validation
  // only run in debug mode
  if (debug_ && (angular_velocities_.size() > 0))
  {
    // shift buffer to remove old measurement
    for (unsigned int i = 0; i < angular_velocities_.size() - 1; i++)
    {
      angular_velocities_[i] = angular_velocities_[i + 1];
      time_step_[i] = time_step_[i + 1];
    }

    // add the new measurements to their buffers
    angular_velocities_[angular_velocities_.size() - 1] = delta_theta_;
    time_step_[time_step_.size() - 1] = delta_t_;

    // take the next element for current measurements
    delta_t_delayed_ = time_step_[0];
    delta_theta_ = angular_velocities_[0];
  }

  // update x and y positions in meters
  x_ += (delta_x_ * cos(theta_ * PI / 180.0) - delta_y_ * sin(theta_ * PI / 180.0));
  y_ += (delta_x_ * sin(theta_ * PI / 180.0) + delta_y_ * cos(theta_ * PI / 180.0));
  if (debug_)
  {
    theta_ = fmod(yaw_angle_ * 180.0 / PI, 360.0);
  }
  else
  {
    theta_ = fmod((theta_ + delta_theta_), 360.0);
  }

  // Two estimations, based on left and right front wheels
  double phi_1;
  double phi_2;
  double velocity_estimate_1;
  double velocity_estimate_2;
  if (steering_angle_ != 0)
  {
    if (steering_angle_ > 0)
    {
      phi_1 = speed_FL_ / (turn_radius_ - width_ / 2);
      phi_2 = speed_FR_ / (turn_radius_ + width_ / 2);
    }
    else
    {
      phi_1 = speed_FR_ / (turn_radius_ - width_ / 2);
      phi_2 = speed_FL_ / (turn_radius_ + width_ / 2);
    }
    velocity_estimate_1 = turn_radius_ * phi_1;
    velocity_estimate_2 = turn_radius_ * phi_2;
  }
  else
  {
    velocity_estimate_1 = speed_FL_;
    velocity_estimate_2 = speed_FR_;
  }

  error_velocity_x_ = .5 * std::abs(speed_FL_ - speed_BL_) + .5 * std::abs(speed_FR_ - speed_BR_);
  error_velocity_theta_ = std::abs(velocity_estimate_1 - velocity_estimate_2);

  // these are the two error metrics published
  // velocity_x_var currently is a constant 0.0569
  velocity_x_var_ = VELOCITY_X_ALPHA * error_velocity_x_ + VELOCITY_X_BETA;
  // function for error_velocity_theta_: -0.6398 * exp(-5.1233 * error_velocity_theta_) + 0.7541
  velocity_theta_var_ = VELOCITY_THETA_ALPHA * exp(VELOCITY_THETA_BETA * error_velocity_theta_) + VELOCITY_THETA_GAMMA;

  nav_msgs::OdometryPtr odom_msg(new nav_msgs::Odometry);
  odom_msg->header.stamp = ros::Time::now();
  // the pose is relative to the header.frame_id reference published
  odom_msg->header.frame_id = "wheel_odom";
  // the twist is relative to the child_fram_id
  odom_msg->child_frame_id = "base_link";

  if (debug_)
  {
    // debug mode publishes data for visualization purposes
    odom_msg->pose.pose.position.x = delta_x_state_estimator_;
    odom_msg->pose.pose.position.y = delta_y_state_estimator_;
    odom_msg->pose.pose.position.z = error_velocity_x_;
    double yaw = theta_ * PI / 180.0;
    odom_msg->pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);

    // Row-major representation of the 6x6 covariance matrix - all covariances that follow take same form
    // The orientation parameters use a fixed-axis representation.
    // In order, the parameters are:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    odom_msg->pose.covariance = {
            10000,  1e-9,  1e-9,  1e-9,  1e-9,  1e-9,
             1e-9, 10000,  1e-9,  1e-9,  1e-9,  1e-9,
             1e-9,  1e-9, 10000,  1e-9,  1e-9,  1e-9,
             1e-9,  1e-9,  1e-9, 10000,  1e-9,  1e-9,
             1e-9,  1e-9,  1e-9,  1e-9, 10000,  1e-9,
             1e-9,  1e-9,  1e-9,  1e-9,  1e-9, 10000
    };

    odom_msg->twist.twist.linear.x = delta_x_ / delta_t_;
    odom_msg->twist.twist.linear.y = delta_y_ / delta_t_;
    odom_msg->twist.twist.linear.z = steering_angle_;

    odom_msg->twist.twist.angular.x = error_velocity_theta_;
    odom_msg->twist.twist.angular.y = delta_theta_state_estimator_;
    // use delayed time for angular z velocity
    odom_msg->twist.twist.angular.z = delta_theta_ * PI / 180.0 / delta_t_delayed_;

    // covariance matrix takes same form as above
    odom_msg->twist.covariance = {
            velocity_x_var_,   1e-9,   1e-9,   1e-9,   1e-9,   1e-9,
                       1e-9, 100000,   1e-9,   1e-9,   1e-9,   1e-9,
                       1e-9,   1e-9, 100000,   1e-9,   1e-9,   1e-9,
                       1e-9,   1e-9,   1e-9, 100000,   1e-9,   1e-9,
                       1e-9,   1e-9,   1e-9,   1e-9, 100000,   1e-9,
                       1e-9,   1e-9,   1e-9,   1e-9,   1e-9, velocity_theta_var_
    };
  }
  else
  {
    odom_msg->pose.pose.position.x = x_;
    odom_msg->pose.pose.position.y = y_;
    odom_msg->pose.pose.position.z = 0;
    double yaw = theta_ * PI / 180.0;
    odom_msg->pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);

    // covariance matrix takes same form as above
    odom_msg->pose.covariance = {
            10000,  1e-9,  1e-9,  1e-9,  1e-9,  1e-9,
             1e-9, 10000,  1e-9,  1e-9,  1e-9,  1e-9,
             1e-9,  1e-9, 10000,  1e-9,  1e-9,  1e-9,
             1e-9,  1e-9,  1e-9, 10000,  1e-9,  1e-9,
             1e-9,  1e-9,  1e-9,  1e-9, 10000,  1e-9,
             1e-9,  1e-9,  1e-9,  1e-9,  1e-9, 10000
    };

    odom_msg->twist.twist.linear.x = delta_x_ / delta_t_;
    odom_msg->twist.twist.linear.y = delta_y_ / delta_t_;
    odom_msg->twist.twist.linear.z = 0;

    odom_msg->twist.twist.angular.x = 0;
    odom_msg->twist.twist.angular.y = 0;
    odom_msg->twist.twist.angular.z = delta_theta_ * PI / 180.0 / delta_t_;

    // covariance matrix takes same form as above
    odom_msg->twist.covariance = {
            velocity_x_var_,   1e-9,   1e-9,   1e-9,   1e-9,   1e-9,
                       1e-9, 100000,   1e-9,   1e-9,   1e-9,   1e-9,
                       1e-9,   1e-9, 100000,   1e-9,   1e-9,   1e-9,
                       1e-9,   1e-9,   1e-9, 100000,   1e-9,   1e-9,
                       1e-9,   1e-9,   1e-9,   1e-9, 100000,   1e-9,
                       1e-9,   1e-9,   1e-9,   1e-9,   1e-9, velocity_theta_var_
    };
  }
  odom_.publish(odom_msg);
}
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "WheelOdometry");
  autorally_core::WheelOdometry wheelOdometry;
  ros::spin();
  return 0;
}
