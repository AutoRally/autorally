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
 * @file wheel_odometry.h
 * @author Justin Zheng <justinyzheng@gmail.com>
 * @date August 8, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief Odometry node
 *
 * @details Uses wheel speeds and servo commands to estimate linear velocities and yaw rate
 **/
#ifndef WHEEL_ODOMETRY_H_
#define WHEEL_ODOMETRY_H_

#include <math.h>

#include "ros/ros.h"
#include <autorally_msgs/chassisState.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <nav_msgs/Odometry.h>

namespace autorally_core
{
/**
* @class WheelOdometry
* @brief Odometry node that uses wheel speeds and servo values
*
* The program reads autorally_msgs::servoMSG and autorally_msgs::wheelSpeeds to estimate the vehicle's linear velocities
* and yaw rate. The program also calculates an error value associated with the difference between the node's output and
* the actual movement of the vehicle.
*
* This program publishes:
* - nav_msgs::Odometry messages with the vehicle's position, orientation, and linear and angular velocities with their
* respective covariances
*/
class WheelOdometry
{
public:
  /**
    * Contructor that recieves parameters and initializes subscribers and publishers
    */
  WheelOdometry();
  ~WheelOdometry();

private:
  const double PI = 3.14159265; ///< Value for pi
  const double MAX_SERVO_VAL = 0.65; ///< Maximum servo value vehicle will steer
  const double STEERING_ALPHA = -21.0832; ///< Coefficient for calculating steering angle
  const double STEERING_BETA = -0.1235; ///< Coefficient for calculating steering angle
  const double VELOCITY_X_ALPHA = 0; ///< Coefficient for calculating variance in x velocity
  const double VELOCITY_X_BETA = 0.569; ///< Coefficient for calculating variance in x velocity
  const double VELOCITY_THETA_ALPHA = -3.199; ///< Coefficient for calculating variance in yaw rate
  const double VELOCITY_THETA_BETA = -5.1233; ///< Coefficient for calculating variance in yaw rate
  const double VELOCITY_THETA_GAMMA = 3.7705; ///< Coefficient for calculating variance in yaw rate

  ros::Subscriber servo_sub_;           ///< Subscriber for servo status
  ros::Subscriber wheel_speeds_sub_;     ///< Subscriber for wheel speeds
  ros::Subscriber state_estimator_sub_;  /// < Subscriber for state estimate
  ros::Publisher odom_;                 ///< Publisher for odometry values

  bool debug_;            ///< Publish debug values when true
  bool using_sim_;        ///< True if simulator is being used
  double time_delay_;     ///< Delay for the angular velocity to account for platform response time
  double max_servo_val_;  ///< Maximum servo value that vehicle can steer to

  double length_;  ///< Length of vehicle wheel base
  double width_;   ///< Length of vehicle axle

  double servo_val_;  ///< Servo value

  double steering_angle_;  ///< Steering angle of vehicle
  double turn_radius_;     ///< Turn radius of the vehicle's immediate path

  double speed_FL_;   ///< Speed of the front left wheel in m/s
  double speed_FR_;   ///< Speed of the front right wheel in m/s
  double speed_BL_;   ///< Speed of the back left wheel in m/s
  double speed_BR_;   ///< Speed of the back right wheel in m/s
  double avg_speed_;  ///< Average speed of the front two wheels in m/s

  double x_;      ///< X position of vehicle relative to initial pose
  double y_;      ///< Y position of vehicle relative to initial pose
  double theta_;  ///< Heading of vehicle relative to initial pose

  double delta_x_;      ///< X velocity in local frame in m/s
  double delta_y_;      ///< Y velocity in local frame in m/s
  double delta_theta_;  ///< Yaw rate in local frame in rad/s
  double yaw_angle_;    ///< Heading in global frame from state estimator

  double delta_t_;          ///< Time difference between the two most recent wheelSpeeds messages
  double delta_t_delayed_;  ///< Delayed deltaT_ values to account for time delay between servo command and vehicle
  /// response
  double prev_;  ///< Timestamp of previous message

  double delta_x_state_estimator_;      ///< X velocity in global frame from state estimate in m/s
  double delta_y_state_estimator_;      ///< Y velocity in global frame from state estimate in m/s
  double delta_theta_state_estimator_;  ///< Yaw rate in global frame from state estimate in m/s

  double error_velocity_x_;  ///< Error value proportional to difference between estimated x velocity and true x
  /// velocity
  double error_velocity_theta_;  ///< Error value proportional to difference between estimated yaw rate and true yaw
  /// rate
  double velocity_x_var_;      ///< Variance of the difference between estimated and true x velocities
  double velocity_theta_var_;  ///< Variance of the difference between estimated and true yaw velocities

  // both of the following are used as buffers for delaying to account for vehicle response time
  std::vector<double> angular_velocities_;  ///< Vector to hold most recent estimated angular velocities
  std::vector<double> time_step_;  ///< Vector to hold most recent timesteps between wheelSpeeds messages received

  bool initial_pose_received_ = false;  ///< First pose received from state estimator(debug)

  /**
    * This callback estimates the steering angle based on incoming servo values
    * @param servo message containing servo steering value
    */
  void servoCallback(const autorally_msgs::chassisStateConstPtr& servo);

  /**
    * This callback estimates the vehicle's linear velocities and yaw rate and publishes an Odometry message. This
    * method calculates error values proportional to the difference between estimated velocities and true velocities
    * @param speed message containing the speed of each wheel in m/s
    */
  void speedCallback(const autorally_msgs::wheelSpeedsConstPtr& speed);

  /**
    * This callback calculates the vehicle's heading and local velocities based on the orientation provided by the state
    * estimator
    * @param SEmsg Odometry message from the state estimator
    */
  void stateEstimatorCallback(const nav_msgs::OdometryConstPtr& state_estimator_msg);
};
}
#endif
