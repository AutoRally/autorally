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

#include "param_getter.h"

#include <autorally_control/ddp/util.h>
#include <autorally_msgs/chassisCommand.h>
#include <autorally_msgs/chassisState.h>
#include <autorally_msgs/runstop.h>
#include <autorally_msgs/pathIntegralStatus.h>
#include <autorally_msgs/pathIntegralTiming.h>
#include <autorally_msgs/neuralNetModel.h>
#include <autorally_control/PathIntegralParamsConfig.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <eigen3/Eigen/Dense>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <atomic>

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

  float last_heading_ = 0.0;
  float throttleMax_ = 0.99;
  int heading_multiplier_ = 0;

	boost::mutex access_guard_;
  std::string nodeNamespace_;

  bool new_model_available_;
  cv::Mat debugImg_;

  bool solutionReceived_ = false;
  bool is_nodelet_;
  std::vector<float> controlSequence_;
  std::vector<float> stateSequence_;
  util::EigenAlignedVector<float, 2, 7> feedback_gains_;
  ros::Time solutionTs_;

  int numTimesteps_;
  double deltaT_;

  double optimizationLoopTime_;

  /**
  * @brief Constructor for AutorallyPlant, takes the a ros node handle and initalizes
  * publishers and subscribers.
  * @param mppi_node A ros node handle.
  */
	AutorallyPlant(ros::NodeHandle global_node, ros::NodeHandle mppi_node, 
                 bool debug_mode, int hz, bool nodelet);

	AutorallyPlant(ros::NodeHandle global_node, bool debug_mode, int hz):AutorallyPlant(global_node, global_node, debug_mode, hz, false){};

  /**
  * @brief Callback for /pose_estimate subscriber.
  */
	void poseCall(nav_msgs::Odometry pose_msg);

  /**
  * @brief Callback for recording the current servo input.
  */
  void servoCall(autorally_msgs::chassisState servo_msg);

  bool hasNewModel();
  virtual void modelCall(autorally_msgs::neuralNetModel model_msg);
  virtual void getModel(std::vector<int> &description, std::vector<float> &data);

  /**
  * @brief Callback for safe speed subscriber.
  */
	void runstopCall(autorally_msgs::runstop safe_msg);

  /**
  * @brief Publishes the controller's nominal path.
  */
	void pubPath(const ros::TimerEvent&);

  void setSolution(std::vector<float> traj, std::vector<float> controls, 
                   util::EigenAlignedVector<float, 2, 7> gains,
                   ros::Time timestamp, double loop_speed);

  void setDebugImage(cv::Mat img);

  void setTimingInfo(double poseDiff, double tickTime, double sleepTime);

  void pubTimingData(const ros::TimerEvent&);

  /**
  * @brief Publishes a control input. 
  * @param steering The steering command to publish.
  * @param throttle The throttle command to publish.
  */
	void pubControl(float steering, float throttle);

  void pubStatus(const ros::TimerEvent&);

  /**
  * @brief Returns the current state of the system.
  */
	AutorallyPlant::FullState getState();

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

  void dynRcfgCall(autorally_control::PathIntegralParamsConfig &config, int lvl);

  bool hasNewDynRcfg();

  autorally_control::PathIntegralParamsConfig getDynRcfgParams();

  virtual void displayDebugImage(const ros::TimerEvent&);

  virtual bool hasNewObstacles(){return false;};
  virtual void getObstacles(std::vector<int> &description, std::vector<float> &data){};

  virtual bool hasNewCostmap(){return false;};
  virtual void getCostmap(std::vector<int> &description, std::vector<float> &data){};

  virtual void shutdown();

protected:
  //SystemParams mppiParams_;
  int poseCount_ = 0;
  bool useFeedbackGains_ = false;
  std::atomic<bool> receivedDebugImg_;
  std::atomic<bool> debugShutdownSignal_;
  std::atomic<bool> debugShutdownSignalAcknowledged_;
  autorally_msgs::neuralNetModel dynamicsModel_;
  autorally_control::PathIntegralParamsConfig costParams_;
  bool hasNewCostParams_ = false;

  const double TIMEOUT = 0.5; ///< Time before declaring pose/controls stale. 

  FullState full_state_; ///< Full state of the autorally vehicle.

  int hz_; ///< The frequency of the control publisher.

  int status_; ///< System status
  std::string ocs_msg_; ///< Message to send to the ocs.

  bool safe_speed_zero_; ///< Current value of safe speed.
  bool debug_mode_; ///< Whether or not the system is in debug/simulation mode.
  bool activated_; ///< Whether or not we've received an initial pose message.

  ros::Time last_pose_call_; ///< Timestamp of the last pose callback.

  ros::Publisher control_pub_; ///< Publisher of autorally_msgs::chassisCommand type on topic servoCommand.
  ros::Publisher status_pub_; ///< Publishes the status (0 good, 1 neutral, 2 bad) of the controller
  ros::Publisher subscribed_pose_pub_; ///< Publisher of the subscribed pose
  ros::Publisher path_pub_; ///< Publisher of nav_mags::Path on topic nominalPath.
  ros::Publisher timing_data_pub_;
  ros::Subscriber pose_sub_; ///< Subscriber to /pose_estimate.
  ros::Subscriber servo_sub_;
  ros::Subscriber model_sub_;
  ros::Timer pathTimer_;
  ros::Timer statusTimer_;
  ros::Timer debugImgTimer_;
  ros::Timer timingInfoTimer_;

  nav_msgs::Path path_msg_; ///< Path message for publishing the planned path.
  geometry_msgs::Point time_delay_msg_; ///< Point message for publishing the observed delay.
  autorally_msgs::pathIntegralStatus status_msg_; ///<pathIntegralStatus message for publishing mppi status
  autorally_msgs::pathIntegralTiming timingData_; ///<pathIntegralStatus message for publishing mppi status
};

}

#endif /* AUTORALLY_PLANT_H_ */
