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
 * @file autorally_plant.h
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Class definition for AutorallyPlant class.
 ***********************************************/

#ifndef AUTORALLY_PLANT_H_
#define AUTORALLY_PLANT_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <autorally_msgs/chassisCommand.h>
#include <autorally_msgs/chassisState.h>
#include <autorally_msgs/runstop.h>
#include <autorally_msgs/pathIntegralStatus.h>

#include <eigen3/Eigen/Dense>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace autorally_control {

/**
* @class AutorallyPlant autorally_plant.h
* @brief Publishers and subscribers for the autorally control system.
* 
* This class is treated as the plant for the MPPI controller. When the MPPI
* controller has a control it sends to a function in this class to get
* send to the actuators. Likewise it calls functions in this class to receive
* state feedback. This class also publishes trajectory and spray information
* and status information for both the controller and the OCS.
*/

class AutorallyPlant //: public Diagnostics  
{
public:
  static const int AUTORALLY_STATE_DIM = 7;
  static const int AUTORALLY_CONTROL_DIM = 2;
  //Struct for holding the autorally pose.
  typedef struct
  { 
    //X-Y-Z position
    float x_pos;
    float y_pos;
    float z_pos;
    //1-2-3 Euler angles
    float roll;
    float pitch;
    float yaw;
    //Quaternions
    float q0;
    float q1;
    float q2;
    float q3;
    //X-Y-Z velocity.
    float x_vel;
    float y_vel;
    float z_vel;
    //Body frame velocity
    float u_x;
    float u_y;
    //Euler angle derivatives
    float yaw_mder;
    //Current servo commands
    float steering;
    float throttle;
  } FullState;

	boost::mutex access_guard_;
  bool new_model_available_;

  std::vector<float> controlSequence_;
  std::vector<float> stateSequence_;
  ros::Time solutionTs_;

  int numTimesteps_;
  double deltaT_;

  /**
  * @brief Constructor for AutorallyPlant, takes the a ros node handle and initalizes
  * publishers and subscribers.
  * @param mppi_node A ros node handle.
  */
	AutorallyPlant(ros::NodeHandle mppi_node, bool debug_mode, int hz);

  /**
  * @brief Callback for /pose_estimate subscriber.
  */
	void poseCall(nav_msgs::Odometry pose_msg);

  /**
  * @brief Callback for recording the current servo input.
  */
  void servoCall(autorally_msgs::chassisState servo_msg);

  /**
  * @brief Callback for safe speed subscriber.
  */
	void runstopCall(autorally_msgs::runstop safe_msg);

  void pubPath(float* nominal_traj, int num_timesteps, int hz);
  /**
  * @brief Publishes the controller's nominal path.
  */
	void pubPath(float* nominal_traj, ros::Publisher path_pub, int num_timesteps, int hz);

  void setSolution(std::vector<float> traj, std::vector<float> controls, ros::Time timestamp);

  /**
  * @brief Publishes a control input. 
  * @param steering The steering command to publish.
  * @param throttle The throttle command to publish.
  */
	void pubControl(float steering, float throttle);

  void pubStatus();

  /**
  * @brief Returns the current state of the system.
  */
	FullState getState();

  /**
  * @brief Returns whether or not the state updater has gone stale or not.
  */
  bool getStale();

  /**
  * @brief Returns the current value of safe speed.
  */
  bool getRunstop();

  /**
  * @brief Returns the timestamp of the last pose callback.
  */
  ros::Time getLastPoseTime();

  /**
  * @brief Checks the system status.
  * @return An integer specifying the status. 0 means the system is operating
  * nominally, 1 means something is wrong but no action needs to be taken,
  * 2 means that the vehicle should stop immediately.
  */
  int checkStatus();


  std::vector<float> getModelParams();

private:
  const double TIMEOUT = 0.5; ///< Time before declaring pose/controls stale. 

  FullState full_state_; ///< Full state of the autorally vehicle.

  int hz_; ///< The frequency of the control publisher.

  int status_; ///< System status
  std::string ocs_msg_; ///< Message to send to the ocs.

  bool safe_speed_zero_; ///< Current value of safe speed.
  bool debug_mode_; ///< Whether or not the system is in debug/simulation mode.
  bool activated_; ///< Whether or not we've received an initial pose message.

  ros::Time last_check_; //Timestamp of the last published control.
  ros::Time last_pose_call_; ///< Timestamp of the last pose callback.

  ros::Publisher control_pub_; ///< Publisher of autorally_msgs::chassisCommand type on topic servoCommand.
  ros::Publisher status_pub_; ///< Publishes the status (0 good, 1 neutral, 2 bad) of the controller
  ros::Publisher delay_pub_; ///< Publisher of geometry::msgs::Point on topic mppiTimeDelay.
  ros::Publisher default_path_pub_; ///< Publisher of nav_mags::Path on topic nominalPath.
  ros::Subscriber pose_sub_; ///< Subscriber to /pose_estimate.
  ros::Subscriber servo_sub_;

  nav_msgs::Path path_msg_; ///< Path message for publishing the planned path.
  geometry_msgs::Point time_delay_msg_; ///< Point message for publishing the observed delay.
  autorally_msgs::pathIntegralStatus status_msg_; ///<pathIntegralStatus message for publishing mppi status
  std::vector<float> model_params_; ///< Array for holding the updated model parameters

};

}

#endif /* AUTORALLY_PLANT_H_ */
