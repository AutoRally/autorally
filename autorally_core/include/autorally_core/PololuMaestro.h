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
 * @file PololuMaestro.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 27, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief Interface for the Pololu Micro Maestro 6 channel servo controller
 *
 * @details This file contains the PololuMaestro class
 ***********************************************/
#ifndef POLOLU_MAESTRO_H_
#define POLOLU_MAESTRO_H_

#include <iostream>
#include <stdio.h>
#include <string>
#include <queue>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ros/time.h>
#include <autorally_core/SerialInterfaceThreaded.h>

#warning autorally_core/ArduinoOnboard.h has been deprecated, refer to autorally_chassis

#define MAX_CONTROL_VALUE 254 ///< Allowable control values are [0-254]
#define SERVO_NEUTRAL_VALUE 127 ///< Middle value that puts servo in neutral
#define SERVO_HOME_POSITION 255 ///< Sends the servo to its 'home' position

/**
 *  @class PololuMaestro PololuMaestro.h
 *  "PololuMaestro/PololuMaestro.h"
 *  @brief Interact with a Pololu Micro Maestro servo controller
 *
 *  Functionality includes setting servo positions and receiving feedback as
 *  to their actual position. Can also retrieve errors from the controller.
 *  Receives commands and broadcasts status using ServoControllerMSG. This
 *  class is currently set up to control a steering servo connected to port
 *  0, a throttle/backBrake servo connected to port 1, and a front brake
 *  connected to port 2. It is easily extendable to control any servo outputs
 *  available. The class also publishes ros diagnostic messages on the
 *  "diagnostics" topic.
 *  Steering values are [-127, 127], -127 is full left, 127 is full right.
 *  Throtttle/back brake values are [-127, 127] 127 is full throttle, -127 is
 *  full back brake.
 *  Front brak vales are from [0-127] 127 is full front brake.
 *
 *  @see http://www.pololu.com/docs/0J40 for documentation of the servo
 *       controller protocol
 *
 */
class PololuMaestro
{
 public:
  /**
   * @brief Constructor requires all configuration needed to connect to controller
   * and properly control connected servos
   *
   */
  ROS_DEPRECATED PololuMaestro();

  ~PololuMaestro();

  void init(ros::NodeHandle &nh, const std::string& nodeName, const std::string& port);

  /**
   * @brief Sends a low level set command over serial to set the position of a servo
   *
   * @param channel which channel (throttle or steering) should be set
   * @param target value [0-254] for position left to right in range,
   *        255 to go to 'home' position
   */
  void setTarget(const unsigned char channel, const unsigned char target);

  /**
   * @brief Sends a low level set command over serial to set the position of a servo
   *
   * @param channel which channel (throttle or steering) should be set
   * @param target pulse width in 1/4 ms, for example, to set the pulse width to
   *        1500ms, give a target value of 1500*4=6000
   */
  void setTargetMS(const unsigned char channel, const unsigned int target);

  /**
   * @brief Sends a low level get command over serial to set the position of a servo
   *
   * @param channel on servo controller to read from
   * @return Value [0-254] of requested channel (127 is neutral)
   */
  unsigned short getTarget(const unsigned char channel);

  /**
   * @brief Retrieve any existing error from control board
   */
  void getErrors();

  SerialInterfaceThreaded m_serialPort;

 private:

};

#endif //POLOLU_MAESTRO_H_
