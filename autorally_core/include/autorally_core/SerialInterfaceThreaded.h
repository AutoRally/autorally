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
 * @file SerialInterfaceThreaded.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief SerialInterfaceThreaded class definition
 *
 ***********************************************/
#ifndef SERIAL_INTERFACE_THREADED_H_
#define SERIAL_INTERFACE_THREADED_H_

#include <autorally_core/SerialCommon.h>

#include <fstream>
#include <queue>
#include <string>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>

#include <ros/ros.h>

/**
 *  @class SerialInterfaceThreaded SerialInterfaceThreaded.h
 *  "infrastrucutre/SerialInterfaceThreaded.h"
 *  @brief Interact with a device on the serial port
 *
 *  Provides the ability to open and interact with a serial device
 *  Includes functionality to automatically accumulate incoming data via a read
 *  thread that relies on select() to minimize wasted compute cycled. The read
 *  data is avaiable in m_data.
 *  @note locking operations are provided to ensure thread-safe data access,
 *        operations, but the user must ensure they call lock() and unlock()
 */
class SerialInterfaceThreaded : public SerialCommon
{

 public:
  std::string m_data; ///< incoming data buffer

  /**
    * @brief SerialInterfaceThreaded default contructor
    */
  SerialInterfaceThreaded();

  /**
    * @brief SerialInterfaceThreaded contructor
    * @param nh NodeHandle used to register stuff
    * @param nodeName name of the parent process
    * @param hardwareID some string to identify the corresponding hardware
    * @param port serial port to connect to
    * @param queueData whether to automatically queue data from port
    *
    * Connects to the specified serial device, sets up diagnostics, starts
    * polling timer if required
    */
  SerialInterfaceThreaded(ros::NodeHandle& nh,
                          const std::string& nodeName,
                          const std::string& hardwareID,
                          const std::string& port,
                          const bool queueData);
  ~SerialInterfaceThreaded();

  void init(ros::NodeHandle& nh,
            const std::string& nodeName,
            const std::string& portHandle,
            const std::string& hardwareID,
            const std::string& port,
            const bool queueData);

  /**
    * @brief Locks internal mutex to ensure exclusive access to data
    */
  void lock();

  /**
    * @brief Attempts to Lock internal mutex to ensure exclusive access to data
    * @return bool If lock acquisition was successful
    */
  bool tryLock();

  /**
    * @brief Unocks internal data mutex (must always be called after lock())
    */
  void unlock();

  typedef boost::function<void()> DataCallback;

  /**
    * @brief Register callback to trigger when data arrives
    * @param callback the function to call that has the signature void funct()
    *
    * The callback is fired in the same thread as the data is read from.
    */
  void registerDataCallback(DataCallback callback);

  void clearDataCallback();

  /**
    * @brief Waits (sleeps thread) until data arrives
    *
    * This mechanism should be used if sometheing other than the read thread
    * needs to know when data has arrived. For instance, if a specific reply
    * is expected right after data is sent, waitForData can be used to
    * sleep the parent thread until the reply has been accumulated by the read
    * thread.
    * @note If no data arrives, this version of waitForData will never return
    */
  void waitForData();

  /**
  * @brief Writes given data to file descriptor (threadsafe)
  * @param data information to be pushed to file descriptor
  *
  * @return int -1 if write failed, number of bytes written if succeeded
  *
  * Overrides SerialCommon:writePort from base class to make a it
  * Thread-safe version
  */
  int writePort(const std::string data);

  /**
    * @brief Writes given data to file descriptor (threadsafe)
    * @param data as a series of bits to be pushed to the descriptor
    * @param length the number of bytes of data to be written to the port
    *
    *@return int -1 if write failed. otherwise, number of bytes written
    *
    * Overrides SerialCommon:writePort from base class to make a it
    * Thread-safe version
    */
  int writePort(const unsigned char* data, unsigned int length);

  /**
    * @brief Tries to write given data to file descriptor (threadsafe)
    * @param data information to be pushed to file descriptor
    *
    * @return int -1 if write failed or could not secure exclusive write access,
    *             otherwise number of bytes written if succeeded
    *
    * Uses try-lock instead of lock in SerialCommon:writePort, so it is earier
    * to avoid race conditions
    */
  int writePortTry(const std::string data);

  /**
    * @brief Tried to write given data to file descriptor (threadsafe)
    * @param data as a series of bits to be pushed to the descriptor
    * @param length the number of bytes of data to be written to the port
    *
    * @return int -1 if write failed or could not secure exclusive write access,
    *             otherwise number of bytes written if succeeded

    *
    * Uses try-lock instead of lock in SerialCommon:writePort, so it is earier
    * to avoid race conditions
    */
  int writePortTry(const unsigned char* data, unsigned int length);


 private:
  std::string m_port; ///< Serial port to connect to
  bool m_settingsApplied; ///< Whether serial settings were successfully applied
  boost::shared_ptr<boost::thread> m_runThread; ///< pointer to the read thread
  boost::mutex m_dataMutex; ///< mutex for accessing incoming data
  boost::mutex m_writeMutex; ///< mutex for writing serial data
  boost::mutex m_waitMutex; ///< mutex for thread synchronization
//  boost::condition_variable m_waitCond; ///< condition variable to wait for data
  DataCallback m_dataCallback; ///< Callback triggered when new data arrives
  volatile bool m_alive;
  /**
    * @brief Function run as a thread that accumulates incoming data
    */
  void run();

  /**
    * @brief Timer triggered callback to publish a diagnostic message
    * @param time information about callback execution
    */
  void diagnosticStatus(const ros::TimerEvent& time);
};
#endif //SERIAL_INTERFACE_THREADED_H_
