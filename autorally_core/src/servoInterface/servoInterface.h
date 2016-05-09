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
 * @file servoInterface.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date July 10, 2013
 * @copyright 2013 Georgia Institute of Technology
 * @brief Interface for servo controller
 *
 * @details This file contains the ServoInterface class
 ***********************************************/
#ifndef SERVO_INTERFACE_H_
#define SERVO_INTERFACE_H_

#include <map>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>

//#include <autorally_msgs/servoControllerMSG.h>
#include <autorally_msgs/servoMSG.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_core/PololuMaestro.h>
#include <autorally_core/SafeSpeed.h>
//#include <autorally_core/RingBuffer.h>

namespace autorally_core
{

class ServoInterface : public nodelet::Nodelet
{
 public:
  struct BrakeSetup {
    bool coupledWithThrottle;
    bool independentFront;
    bool independentBack;
  };

  struct ServoSettings
  {
    unsigned short center; ///< calibrated zero of servo in us
    unsigned short min;    ///< calibrated minimum of servo in us (left)
    unsigned short max;    ///< calibrated maximum of servo in us (right)
    unsigned short range;  ///< range of servo signal (max-min)
    unsigned char port;    ///< port on servo controller
    bool reverse;          ///< if the servo should be reversed

    // center should be (min+max)/2
    ServoSettings():
      center(1500),
      min(1050),
      max(1950),
      range(max-min),
      port(0),
      reverse(false)
    {}
  };

  ~ServoInterface();

  virtual void onInit();

 private:
  std::map<std::string, ros::Subscriber> m_servoSub;
  ros::Subscriber m_speedCommandSub;
//  ros::Subscriber m_irDataSub;
  ros::Publisher m_servoMSGPub; ///<Publisher for servoInterfaceStatus
  ros::Timer m_throttleTimer; ///<Timer to trigger throttle set
//  ros::Timer m_servoStatusTimer; ///<Timer to trigger status message publishing
  PololuMaestro m_maestro; ///< Local instance connected to the hardware
  SafeSpeed m_ss; ///< Local instance used to computer the safe speed value

  double m_speedCommand; ///< Speed someone wants the vehicle to go
//  signed char m_previousThrottleValue; ///< Value used for PD controller

//  autorally_msgs::servoMSG m_servoMSG; ///<Current command message
//  autorally_msgs::servoMSGPtr m_servoMSGStatus; ///<Local status message
  std::map<std::string, ServoSettings> m_servoSettings;
  BrakeSetup m_brakeSetup;
  double m_servoCommandMaxAge;
//  std::string m_steeringCommander;
//  std::string m_throttleCommander;

  struct priorityEntry
  {
    std::string id;
    int priority;
  };
  
  struct priorityComparator
  {
    bool operator() (const priorityEntry& a, const priorityEntry& b)
    {
      return a.priority < b.priority;
    }
  };
  
  std::map<std::string, autorally_msgs::servoMSG> m_servoCommandMsgs;
  std::vector<priorityEntry> m_servoCommandPriorities;

  /**
   * @brief Timer triggered callback to publish a servoInterfaceStatus message
   * @param time information about callback execution
   */
  //void servoStatusTimerCallback(const ros::TimerEvent& time);

  /**
   * @brief Callback for receiving control messages
   * @see autorally_core::servoMSG
   * @param msg the message received from ros comms
   */
  void servoMSGCallback(const autorally_msgs::servoMSGConstPtr& msg);

  /**
   * @brief Callback for receiving speed command messages
   * @param msg the message received from ros comms
   */
  void speedCallback(const std_msgs::Float64ConstPtr& msg)
    {m_speedCommand = msg->data;}

  /**
   * @brief Time triggered callback to set position of throttle servo
   * @param time information about callback firing
   */
  void setServos(const ros::TimerEvent& time);

  /**
   * @brief Retrieves servo positiona from control board and scales them to 0-254
   */
  void populateStatusMessage(autorally_msgs::servoMSGPtr& status);

  /**
   * @brief
   *
   * @param channel which channel (throttle or steering) should be set
   * @param target value to set channel to (-100.0 to 100.0)
   */
  bool setServo(const std::string &channel, const double target);
  bool getServo(const std::string &channel, double& position);
  
  void loadServoParams();
  void loadServoCommandPriorities();
};

}//end infratructure
#endif //SERVO_INTERFACE_H_
