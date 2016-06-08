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
 * @file ConstantSpeedController.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date November 13, 2013
 * @copyright 2012 Georgia Institute of Technology
 * @brief ConstantSpeedController class definition
 *
 ***********************************************/
#ifndef JUMP_CONTROL_H_
#define JUMP_CONTROL_H_

#include <map>

#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

#include <autorally_msgs/servoMSG.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/safeSpeed.h>
#include <autorally_core/RingBuffer.h>
//#include <autorally_core/TimeTaggedRingBuffer.h>

namespace autorally_control
{

class ConstantSpeedController : public nodelet::Nodelet
{
 public:
  ConstantSpeedController();
  ~ConstantSpeedController();
  void onInit();

 private:
  enum SpeedControllerState
  {
    DISABLED,
    SETUP,
    ACCELERATING,
    MAINTAINING_SPEED,
  };

  SpeedControllerState m_controllerState;
  ros::Subscriber m_safeSpeedSub;
  ros::Subscriber m_wheelSpeedsSub;
  ros::Publisher m_servoMSGPub; ///<Publisher for servoCommands
  ros::Timer m_controlTimer;
  ros::Timer m_controlEnableTimer;
  std::string m_speedCommander;

  double m_constantSpeedPrevThrot; ///< Difference between desired and actual velocity from previous iteration
//  double m_safeSpeedISum; ///< Accumulated velocity error
  double m_constantSpeedKP; ///< Scaler for the proportion component
  double m_constantSpeedKD; ///< Scaler for the derivative component
  double m_constantSpeedKI; ///< Scaler for the integral component
  double m_constantSpeedIMax; ///< Maximum Integral Error
  double m_integralError;
//  double m_throttleOffset;
//  double m_safeSpeedMultiplier;

  bool m_controlEnabled;
  double m_accelerationRate;
  double m_frontWheelsSpeed;
  double m_backWheelsSpeed;
  autorally_msgs::safeSpeed m_mostRecentSafeSpeed;
  autorally_msgs::servoMSGPtr m_servoMSG; ///<Current command message
  double m_speedSetPoint;
  double m_throttleAccStart;
  double m_throttleAccEnd;
  ros::Time m_accelerationStartTime;
  double m_accelerationDuration;

  autorally_core::RingBuffer<double> m_throttleMappings;
  std::vector<double> m_accelerationProfile;
  void safeSpeedCallback(const autorally_msgs::safeSpeedConstPtr& msg);
  void wheelSpeedsCallback(const autorally_msgs::wheelSpeedsConstPtr& msg);

  void enableControlCallback(const ros::TimerEvent& time);
  void controlCallback(const ros::TimerEvent& time);

  std::vector<double> generateAccelerationProfile(const int count);
  void loadThrottleCalibration();

};

}
#endif 
