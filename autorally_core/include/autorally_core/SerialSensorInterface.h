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
 * @file SerialSensorInterface.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief SerialSensorInterface class definition
 *
 ***********************************************/
#ifndef SERIAL_SENSOR_INTERFACE_H_
#define SERIAL_SENSOR_INTERFACE_H_

#include <autorally_core/SerialCommon.h>

#include <fstream>
#include <queue>
#include <string>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

/**
 *  @class SerialSensorInterface SerialSensorInterface.h
 *  "autorally_core/SerialSensorInterface.h"
 *  @brief Interact with a device on the serial port
 *
 *  Provides the ability to connect to a specified serial device at 57600 baud
 *  with no parity, one stop bit, 8 data bits, no flow control. Includes
 *  functionality to automatically accumulate incoming data by polling. The read
 *  data is avaiable in m_data. Automatically sets up basic diagnostics based on
 *  the provided constructor parameters.
 *  @note The serial port setting configuraiton was taken loosely from:
 *  http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
 */
class SerialSensorInterface : public SerialCommon
{

 public:
  std::string m_data; ///< incoming data buffer

  /**
    * @brief SerialSensorInterface contructor
    *
    * Creates a dummy interface. Sets the portID as -1,
    * does not set up diagnostics
    */
  SerialSensorInterface();

  /**
    * @brief SerialSensorInterface contructor
    * @param nh NodeHandle used to register stuff
    * @param nodeName name of the parent process
    * @param hardwareID some string to identify the corresponding hardware
    * @param port serial port to connect to
    * @param queueData whether to automatically queue data from port
    *
    * Connects to the specified serial device, sets up diagnostics, starts
    * polling timer if required
    */
  SerialSensorInterface(ros::NodeHandle &nh,
                        const std::string nodeName,
                        const std::string hardwareID,
                        const std::string port,
                        const bool queueData);
  ~SerialSensorInterface();

  void init(ros::NodeHandle &nh,
            const std::string nodeName,
            const std::string portHandle,
            const std::string hardwareID,
            const std::string port,
            const bool queueData);

  /**
    * @brief Reads data from file descriptor
    * @return std::string of data read.
    *
    * If the last character is LF it replaces it with a '\0'.
    */
  std::string readPort();

 private:
  std::string m_port; ///< Serial port to connect to
  bool m_settingsApplied; ///< Whether serial settings were successfully applied
  bool m_queueData; ///< If data should be automatically queued from the port
  ros::Time m_mostRecentData; ///< Time when the most recent data was read
  ros::Timer m_serialTimer; ///<Timer to trigger serial polling

  /**
    * @brief Timer triggered callback to poll file descriptor for available data
    * @param time information about callback execution
    *
    * Any read data is appended to m_data
    */
  void pollSerial(const ros::TimerEvent& time);

  /**
    * @brief Timer triggered callback to publish a diagnostic message
    * @param time information about callback execution
    */
  void diagnosticStatus(const ros::TimerEvent& time);
};
#endif //SERIAL_SENSOR_INTERFACE_H_
