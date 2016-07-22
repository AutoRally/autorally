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

#include <autorally_msgs/chassisCommand.h>
#include <autorally_msgs/chassisState.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/runstop.h>
#include <autorally_core/PololuMaestro.h>

#warning autorally_core/servoInterface.h has been deprecated, refer to autorally_chassis

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
  ros::Subscriber m_runstopSub; ///< Subscriber for all incoming runstop messages
  ros::Publisher m_chassisStatePub; ///<Publisher for servoInterfaceStatus
  ros::Timer m_throttleTimer; ///<Timer to trigger throttle set
  PololuMaestro m_maestro; ///< Local instance connected to the hardware

  std::map<std::string, ServoSettings> m_servoSettings;
  BrakeSetup m_brakeSetup;
  double m_servoCommandMaxAge;

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
  
  std::map<std::string, autorally_msgs::chassisCommand> m_servoCommandMsgs;
  std::vector<priorityEntry> m_servoCommandPriorities;

  ros::Duration m_runstopMaxAge;
  std::map<std::string, autorally_msgs::runstop> m_runstops; ///< Map of the most recently received runstop message from
                                                            ///< all nodes publishing the message

  /**
   * @brief Callback for receiving control messages
   * @see autorally_core::chassisCommand
   * @param msg the message received from ros comms
   */
  void chassisCommandCallback(const autorally_msgs::chassisCommandConstPtr& msg);

   /**
   * @brief Callback for incoming runstop messages
   * @see autorally_core::runstop
   * @param msg the runstop message received from ros comms
   */
  void runstopCallback(const autorally_msgs::runstopConstPtr& msg) {m_runstops[msg->sender] = *msg;}

  /**
   * @brief Time triggered callback to set position of throttle servo
   * @param time information about callback firing
   */
  void setServos(const ros::TimerEvent& time);

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
