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
 * @file CameraTrigger.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 22, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief
 *
 * @details This file contains the CameraTrigger class
 ***********************************************/

#ifndef CAMERA_TRIGGER
#define CAMERA_TRIGGER

#define PI 3.141592653589793238462;

#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
//#include <std_msgs/Float64.h>

#include <autorally_core/SerialInterfaceThreaded.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/chassisCommand.h>
#include <autorally_core/camera_trigger_paramsConfig.h>

#include <boost/tokenizer.hpp>
//#include <boost/circular_buffer.hpp>

#include <stdio.h>
#include <string>

namespace autorally_core
{

/**
 *  @class CameraTrigger CameraTrigger.h
 *  "CameraTrigger/CameraTrigger.h"
 *  @brief Interacts with the onboard Arduino card
 *
 *  The Arduino card publishes data on startup. This class
 *  publishes that data as an arduinoData message.
 */
class CameraTrigger : public nodelet::Nodelet
{

 public:
  /**
   * @brief Constructor requires all configuration needed to connect to arduino
   *
   */
  CameraTrigger();
  ~CameraTrigger();

  void onInit();

 private:
  dynamic_reconfigure::Server<camera_trigger_paramsConfig> m_dynReconfigServer;
  SerialInterfaceThreaded m_port; ///<Serial port for arduino data
  int m_triggerFPS; ///< Frame rate for the camera external trigger

  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  
    /**
   * @brief Process incoming data stream to pull out complete message strings
   * @return bool if a complete message string was found and processed
   */
  void triggerDataCallback();
  bool findMessage(std::string& msg);
  void configCallback(const camera_trigger_paramsConfig &config, uint32_t level);
 
};

}

#endif //CAMERA_TRIGGER
