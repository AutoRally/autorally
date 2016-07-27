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
 * @file RunStop.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date July 20, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief Publishes an autorally_msgs::runstop message based on run stop box state
 *
 * @details This file contains the RunStop class definition
 ***********************************************/
#ifndef RUN_STOP
#define RUN_STOP

#include <ros/ros.h>
#include <ros/time.h>
#include <autorally_core/SerialInterfaceThreaded.h>
#include <autorally_msgs/runstop.h>

#include <iostream>
#include <stdio.h>
#include <string>
#include <queue>

/**
 *  @class RunStop RunStop.h
 *  "RunStop/RunStop.h"
 *  @brief Interacts with Arduino connected to a 4 button run stop box
 *
 *  Provides a runstop message based on the combined state of a 4-button run
 *  stop box. Any red or yellow button pressed will set motionEnabled to false.
 *  The green button must be pressed to set motionEnabled to true.
 */
class RunStop
{

 public:
  ros::Timer doWorkTimer_; ///<Timer to trigger data processing
  ros::Publisher runstopPub_;  ///<Publisher for runstop message

  /**
   * @brief Constructor requires all configuration needed to connect to arduino
   *        as well as runstop values to publish
   *
   * @param nh NodeHandle to use for setup
   * @param port to connect to arduino
   */
  RunStop(ros::NodeHandle &nh,
                const std::string& port);

  ~RunStop();

  /**
   * @brief Process incoming data stream to pull out complete message strings
   * @return bool if a complete message string was found and processed
   */
  bool processData();


  /**
     * @brief Timer triggered callback to publish runstop
     * @param time information about callback execution
     */
  void doWorkTimerCallback(const ros::TimerEvent& time);

 private:
  SerialInterfaceThreaded serialPort_;
  std::string state_; ///< Current run stop button state received from Arduino
  ros::Time lastMessageTime_; ///< Time of most recent message from Arduino
  autorally_msgs::runstop runstopData_; ///< Local runstop message
};
#endif //RUN_STOP
