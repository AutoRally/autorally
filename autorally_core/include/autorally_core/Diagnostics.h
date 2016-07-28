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
 * @file Diagnostics.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief Diagnostics class definition
 *
 ***********************************************/
#ifndef DIAGNOSTICS_H_
#define DIAGNOSTICS_H_

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include <string>
#include <map>

/**
 *  @class Diagnostics Diagnostics.h
 *  "Diagnostics/Diagnostics.h"
 *  @brief Provide hardware diagnostic capabilities
 *
 *  Aggregates and publishes diagnostic information. Uses a ros
 *  diagnostic_updator as the underlying mechanism to publish data. Messages
 *  have a corresponding level indicated by what method is called to queue the
 *  message. Diagnostics are meant for hardware components only.
 *  @note If a diagnostic publishing frequency other than 1s is required, define
 *  diagnosticsFrequency double in the parameter server with desired frequency.
 *  This sets the publish fequency of all diagnostics in the system.
 *  @note If the same message is queued between diagnostics being pubished, only
 *  one will be included in the next diagnostics message sent. There is no
 *  guarantee as to the level of the message if multiple messages were queued
 *  with identical message bodies using different levels (ok, warn, error).
 */
class Diagnostics
{
 public:
  Diagnostics();
  /**
    * @brief Diagnostics constructor
    * @param otherInfo Short string of additional info about node
    * @param hardwareID some string to identify the corresponding hardware
    * @param hardwareLocation how or where the hardware is connected
    *
    * Initializes a diagnostic_updator and sets up a timer to trigger
    * callback publishing at the desired frequency (1s default).
    */
  Diagnostics(const std::string otherInfo,
              const std::string hardwareID,
              const std::string hardwareLocation);
  ~Diagnostics();

  void init(const std::string& otherInfo,
            const std::string& hardwareID,
            const std::string& hardwareLocation);
  /**
    * @brief Send a standard diagnostic message with a key:value
    * @param key the key to queue
    * @param value the value to queue
    * @param lock whether to lock the data mutex, should only be false if you already locked the mutex yoruself
    */
  void diag(const std::string key, const std::string value, bool lock = true);

  /**
    * @brief Send a diagnostic message with level OK
    * @param msg the diagnostic message to queue
    */
  void diag_ok(const std::string msg);

  /**
    * @brief Send a diagnostic message with level WARN
    * @param msg the diagnostic message to queue
    */
  void diag_warn(const std::string msg);

  /**
    * @brief Send a diagnostic message with level ERROR
    * @param msg the diagnostic message to queue
    */
  void diag_error(const std::string msg);

  /**
    * @brief Set the overall diagnostic message to level OK
    */
  void OK();

  /**
    * @brief Set the overall diagnostic message to level WARN
    */
  void WARN();

  /**
    * @brief Set the overall diagnostic message to level ERROR
    */
  void ERROR();

  /**
    * @brief Accumulates frequency information for a named counter
    * @param name The name of the counter to increment
    *
    * Calling tick every time a message is published will result in the
    * publishing frequencies to be displayed in diagnostics. Tick can keep track
    * of and publish multiple counters at once, identified by the message name.
    */
  void tick(const std::string &name);

 private:
  ros::Timer m_heartbeatTimer; ///< Timer to trigger diagnostic publishing
  ros::Timer m_statusTimer; ///<Timer to trigger diagnostics status

  std::string m_hardwareLocation; ///< Location of hardware diagnostics relate to
  diagnostic_updater::Updater m_updater; ///< ros diagnostics publisher
  std::map<std::string, char> m_diagMsgs; ///< map of pending diagnostic messages
  std::map<std::string, std::string> m_diags; ///< map of pending standard messages
  unsigned char m_overallLevel; ///< overall status level of the message
  ///< Holds tick frequency counters
  std::map<std::string, std::vector<std::pair<int, ros::Time> > > m_ticks;

  boost::mutex m_dataMutex; ///< mutex for accessing data

  /**
    * @brief Timer triggered callback to force publishing of diagnostics
    * @param time information about callback execution
    */
  void diagUpdate(const ros::TimerEvent& time);

  /**
    * @brief Callback triggered to form diagnostics message
    * @param stat reference to diagnostic information to be published
    */
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  /**
    * @brief Timer triggered callback to publish a diagnostic message
    * @param time information about callback execution
    */
  virtual void diagnosticStatus(const ros::TimerEvent& time) = 0;
};
#endif //DIAGNOSTICS_H_
