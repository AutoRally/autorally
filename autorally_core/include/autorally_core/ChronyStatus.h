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
 * @file ChronyStatus.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief ChronyStatus class definition
 *
 ***********************************************/
#ifndef CHRONY_STATUS_H_
#define CHRONY_STATUS_H_

#include <autorally_core/Diagnostics.h>

#include <ros/ros.h>
#include <ros/time.h>

#include <stdlib.h>
#include <string>

/**
 *  @class ChronyStatus ChronyStatus.h 
 *  "autorally_core/ChronyStatus.h"
 *  @brief Publish time synchronization diagnostic information from chrony
 *
 */
class ChronyStatus : public Diagnostics
{

 public:  
  /**
    * @brief ChronyStatus constructor
    * @param nh NodeHandle used to register stuff
    * @param nodeName name of the parent process
    * @param hardwareID some string to identify the corresponding hardware
    * @param port serial port to connect to
    *
    * Initializes a node monitor the clock offset values for all clients in the
    * system that are running chrony
    */
  ChronyStatus(ros::NodeHandle &nh,
               const std::string nodeName,
               const std::string hardwareID,
               const std::string port);
  ~ChronyStatus();
  
 private:
  std::string m_masterHostname; ///< hostname of the ros master
  std::string m_hostname; ///< hostname where the program is executing
  ros::Timer m_updateClientsTimer; ///< timer to update clients vector
  std::vector<std::string> m_clients; ///< list of clients setting their time using chrony

  /**
    * @brief Timer triggered callback to update the client list by asking master
    * @param time information about callback execution
    */
  void updateClients(const ros::TimerEvent& time);

  /**
    * @brief Timer triggered callback to publish time offsets of machines in the
    *        system.
    * @param time information about callback execution
    *
    * Polls all known clients of master and publishes the respective estimated
    * clock offsets with respect to ROS master.
    */
  void diagnosticStatus(const ros::TimerEvent& time);
  
};
#endif //CHRONY_STATUS_H_
