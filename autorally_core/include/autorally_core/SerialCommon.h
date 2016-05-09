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
 * @file SerialCommon.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date July 22, 2013
 * @copyright 2013 Georgia Institute of Technology
 * @brief SerialCommon class definition
 *
 ***********************************************/
#ifndef SERIAL_COMMON_H_
#define SERIAL_COMMON_H_

#include <autorally_core/Diagnostics.h>

#include <termios.h>
#include <string>

#include <ros/ros.h>

/**
 *  @class SerialCommon SerialCommon.h
 *  "autorally_core/SerialCommon.h"
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
class SerialCommon : public Diagnostics
{

 public:

  SerialCommon();
  /**
    * @brief SerialCommon contructor
    *
    * Creates a dummy interface. Sets the portID as -1,
    * does not set up diagnostics
    */
  SerialCommon(const std::string& portHandle,
               const std::string& hardwareID,
               const std::string& portPath);

  ~SerialCommon();


  void init(const std::string& portHandle,
            const std::string& hardwareID,
            const std::string& portPath);

  /**
    * @brief Connects to controller with provided information
    * @note if connect() fails, nothing else will work
    * @return bool if connection was successful
    */
  bool connect(const std::string& port,
               const int baud,
               const std::string parity,
               const int stopBits,
               const int dataBits,
               const bool hardwareFlow,
               const bool softwareFlow);

  /**
    * @brief Pushes given data to file descriptor
    * @param data information to be pushed to file descriptor
    *
    * @return int -1 if write failed, number of bytes written if succeeded
    */
  int writePort(const std::string data) const;

  /**
    * @brief Pushes given data to file descriptoor
    * @param data as a series of bits to be pushed to the descriptor
    * @param length the number of bytes of data to be written to the port
    *
    *@return int -1 if write failed. otherwise, number of bytes written
    */
  int writePort(const unsigned char* data, unsigned int length) const;


  /**
    * @brief Check if connected to device
    * @return bool if connected
    */
  bool connected() const {return m_fd != -1;}

  /**
    * @brief Get raw file descriptor for custom interaction with serial device
    * @return int the raw file descriptor
    */
  const int& fileDescriptor() const {return m_fd;}

  /**
    * @brief get any serial settings error from connecting
    * @return std::string error string
    */
  const std::string& getSettingError() const {return m_settingError;}

 private:
  int m_fd; ///< File descriptor of connection
  std::string m_settingError; ///< Serial settings error, if exists
  struct termios m_old_port_settings;
};
#endif //SERIAL_COMMON_H_
