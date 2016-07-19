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
/**********************************************
 * @file autorally_chassis.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date July 10, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief Interface for and AutoRally chassis
 *
 * @details This file contains the autorally_chassis class definition
 ***********************************************/
#ifndef AUTORALLY_CHASSIS_H_
#define AUTORALLY_CHASSIS_H_

#include <map>
#include <vector>
#include <algorithm>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>

#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/runstop.h>
#include <autorally_msgs/chassisCommand.h>
#include <autorally_msgs/chassisState.h>

//#include <autorally_msgs/servoControllerMSG.h>
//#include <autorally_msgs/servoMSG.h>
//#include <autorally_core/PololuMaestro.h>
//#include <autorally_core/SafeSpeed.h>
//#include <autorally_core/RingBuffer.h>

#include <autorally_core/SerialInterfaceThreaded.h>

namespace autorally_core
{

class autorally_chassis : public nodelet::Nodelet
{
 public:
  struct ActuatorConfig
  {
    unsigned short center; ///< calibrated zero of servo in us
    unsigned short min;    ///< calibrated minimum of servo in us (left)
    unsigned short max;    ///< calibrated maximum of servo in us (right)
    unsigned short range;  ///< range of servo signal (max-min)
    unsigned char port;    ///< port on servo controller
    bool reverse;          ///< if the servo should be reversed

    // center should be (min+max)/2
    ActuatorConfig():
      center(1500),
      min(1000),
      max(2000),
      range(max-min),
      port(0),
      reverse(false)
    {}
  };

  ~autorally_chassis();

  virtual void onInit();

 private:
  SerialInterfaceThreaded serialPort_;

  std::map<std::string, ros::Subscriber> chassisCommandSub_;
  ros::Subscriber runstopSub_;

  ros::Publisher chassisStatePub_; ///<Publisher for autorally_chassis Status
  ros::Publisher wheelSpeedsPub_;
  ros::Publisher chassisCommandPub_;

  ros::Timer chassisControlTimer_; ///<Timer to trigger throttle set

  std::map<std::string, ActuatorConfig> actuatorConfig_;
  ros::Duration chassisCommandMaxAge_;
  ros::Duration runstopMaxAge_;

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
  
  std::map<std::string, autorally_msgs::chassisCommand> chassisCommands_;
  std::map<std::string, autorally_msgs::runstop> runstops_;
  std::vector<priorityEntry> chassisCommandPriorities_;

  std::vector<std::pair<std::string, double> > escRegisterData_ =
        { {"ESC Input Voltage", 20.0},
          {"ESC Input Ripple Voltage", 4.0},
          {"Current", 50.0},
          {"Throttle ms", 1.0},
          {"Output Power", 0.2502},
          {"Motor RPM", 20416.66},
          {"Temperature", 50.0},
          {"BEC Voltage", 4.0},
          {"BEC Current", 4.0}, };

  boost::mutex chassisStateMutex_; ///< mutex for accessing incoming data
  bool throttleRelayEnabled_;
  bool autonomousEnabled_;

  /**
   * @brief Callback for receiving control messages
   * @see autorally_core::chassisCommand
   * @param msg the message received from ros comms
   */
  void chassisCommandCallback(const autorally_msgs::chassisCommandConstPtr& msg);


  void runstopCallback(const autorally_msgs::runstopConstPtr& msg) {runstops_[msg->sender] = *msg;}

  void chassisFeedbackCallback();
  void processChassisMessage(std::string msgType, std::string msg);
  /**
   * @brief Callback for receiving speed command messages
   * @param msg the message received from ros comms
   */
  //void speedCallback(const std_msgs::Float64ConstPtr& msg)
  //  {m_speedCommand = msg->data;}

  /**
   * @brief Time triggered callback to set chassis actuators
   * @param time information about callback firing
   */
  void setChassisActuators(const ros::TimerEvent& time);
  
  void sendCommandToChassis(autorally_msgs::chassisStatePtr& state);
  short actuatorCmdToMs(double actuatorValue, std::string actuator);
  double actuatorMsToCmd(int pulseWidth, std::string actuator);


  void loadChassisConfig();
  void loadChassisCommandPriorities();
};

}//end autorally_core
#endif //AUTORALLY_CHASSIS_H_
