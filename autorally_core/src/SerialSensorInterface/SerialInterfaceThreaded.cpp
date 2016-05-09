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
 * @file SerialInterfaceThreaded.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief SerialInterfaceThreaded class implementation
 *
 ***********************************************/
#include <autorally_core/SerialInterfaceThreaded.h>

#include <sys/select.h>

#include <ros/time.h>

SerialInterfaceThreaded::SerialInterfaceThreaded() :
  m_port(""),
  m_settingsApplied(false),
  m_alive(false)
{}

SerialInterfaceThreaded::SerialInterfaceThreaded(ros::NodeHandle& nh,
                                                 const std::string& portHandle,
                                                 const std::string& hardwareID,
                                                 const std::string& port,
                                                 const bool queueData) :
  SerialCommon(portHandle, hardwareID, port),
  m_port(port),
  m_settingsApplied(false),
  m_alive(false)
{
  init(nh, ros::this_node::getName(), portHandle, hardwareID, port, queueData);
}

SerialInterfaceThreaded::~SerialInterfaceThreaded()
{
  /*
  ** std::cout is used here instead of ROS_INFO because
  ** when the destructor is called, ros::shutdown has already been executed
  ** and roscore is not running. So ROS_INFO will not print any output.
  ** Additionally, in the launch file the output param must be set as "screen"
  ** to be able to view the std::cout outputs.
  */
  
  //clearDataCallback();

  //if the serial thread isnt dead yet, wait for it to close
  if(m_alive)
  {
    m_alive = false;
    std::cout << "Joining " << m_port.c_str() << std::endl;  
    m_runThread->join();
    std::cout << "Joined " << m_port.c_str() << std::endl;
  }
  //std::cout << "Shutting down " << m_port.c_str() << " " << close(fileDescriptor()) << std::endl;
}

void SerialInterfaceThreaded::init(ros::NodeHandle& nh,
                                   const std::string& portName,
                                   const std::string& portHandle,
                                   const std::string& hardwareID,
                                   const std::string& port,
                                   const bool queueData)
{
  std::string parity;
  int baud, stopBits, dataBits;
  bool hardwareFlow, softwareFlow;
  m_port = port;


  std::string newP = portName+((portHandle.empty())?"":"/"+portHandle);
  SerialCommon::init(newP, hardwareID, m_port);

  //get current node name to allow access to the serial parameters
  //specific to this node
  //std::string nName = nodeName;//+((portHandle.length()>0)?"/"+portHandle:"");
  //std::cout << portName+((portHandle.empty())?"":"/"+portHandle) << std::endl;

  if(!nh.getParam(newP+"/serialBaud", baud) ||
     !nh.getParam(newP+"/serialParity", parity) ||
     !nh.getParam(newP+"/serialStopBits", stopBits) ||
     !nh.getParam(newP+"/serialDataBits", dataBits) ||
     !nh.getParam(newP+"/serialHardwareFlow", hardwareFlow) ||
     !nh.getParam(newP+"/serialSoftwareFlow", softwareFlow))
  {
    ROS_ERROR("Could not get all SerialInterfaceThreaded parameters for %s", portName.c_str());
  }

  m_settingsApplied = this->connect(m_port,
                              baud,
                              parity,
                              stopBits,
                              dataBits,
                              hardwareFlow,
                              softwareFlow);

  if(queueData && m_settingsApplied)
  {
    //start worker in separate thread
    m_alive = true;
    m_runThread = boost::shared_ptr< boost::thread >
      (new boost::thread(boost::bind(&SerialInterfaceThreaded::run, this)));
  }
}

void SerialInterfaceThreaded::run()
{
  fd_set rfds;
  struct timeval tv;
  int retval;
  char data[512];
  int received;

  if(!connected())
  {
    diag_error("Not connected, cannot start reading");
    return;
  }

  while(m_alive)
  {
    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&rfds);
    FD_SET(fileDescriptor(), &rfds);

    /* Wait up to one seconds. */
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    retval = select(fileDescriptor()+1, &rfds, NULL, NULL, &tv);
    /* Don't rely on the value of tv now! */

    if(retval == -1)
    {
      std::cout << m_port << " select() error" << std::endl;
      diag_error("select() error");
    }
    else if(retval)
    {
      /* FD_ISSET(0, &rfds) will be true. */
      if( (received = read(fileDescriptor(), &data, 512)) >= 0)
      {
        m_dataMutex.lock();
        m_data.append(data, received);

        m_dataMutex.unlock();
        //callback triggered within same thread
        if(m_dataCallback)
        {
          m_dataCallback();
        }
        //condition can notify (wake) other threads waiting for data
//        m_waitCond.notify_all();
      }
    }
    else
    {
      diag_warn("No data within previous second");
    }
  }

  //since ros is shutdown and ROS diag messages wouldnt go anywhere
  std::cout << "SerialInterfaceThreaded Done Running " << fileDescriptor() << std::endl;
}

void SerialInterfaceThreaded::lock()
{
  m_dataMutex.lock();
}

bool SerialInterfaceThreaded::tryLock()
{
  return m_dataMutex.try_lock();
}

void SerialInterfaceThreaded::unlock()
{
  m_dataMutex.unlock();
}

void SerialInterfaceThreaded::registerDataCallback(DataCallback callback)
{
  m_dataCallback = callback;
}

void SerialInterfaceThreaded::clearDataCallback()
{
  m_dataCallback = NULL;
}

//void SerialInterfaceThreaded::waitForData()
//{
//  boost::unique_lock<boost::mutex> lock(m_waitMutex);
//  m_waitCond.wait(lock);
//}

//bool SerialInterfaceThreaded::waitForData(const long& timeMS)
//{
//  boost::unique_lock<boost::mutex> lock(m_waitMutex);
//  return m_waitCond.timed_wait(lock, boost::posix_time::milliseconds(timeMS));
//}

int SerialInterfaceThreaded::writePort(const std::string data)
{
  if(connected())
  {
    boost::unique_lock<boost::mutex> lock(m_writeMutex);
    return SerialCommon::writePort(data);
  }
  return -1;
}
int SerialInterfaceThreaded::writePort(const unsigned char* data, unsigned int length)
{
  if(connected())
  {
    boost::unique_lock<boost::mutex> lock(m_writeMutex);
    return SerialCommon::writePort(data, length);
  }
  return -1;
}

int SerialInterfaceThreaded::writePortTry(const std::string data)
{
  if(connected())
  {
    boost::unique_lock<boost::mutex> lock(m_writeMutex, boost::try_to_lock);
    if(lock)
    {
      return SerialCommon::writePort(data);
    }
  }
  return -1;
}

int SerialInterfaceThreaded::writePortTry(const unsigned char* data, unsigned int length)
{
  if(connected())
  {
    boost::unique_lock<boost::mutex> lock(m_writeMutex, boost::try_to_lock);
    if(lock)
    {
      return SerialCommon::writePort(data, length);
    }
  }
  return -1;
}

void SerialInterfaceThreaded::diagnosticStatus(const ros::TimerEvent& /*time*/)
{
  //queue up a status messages
  if(!m_settingsApplied)
  {
    diag_error("Serial port setting error: "+getSettingError());
  }
  if(!connected())
  {
    diag_error("Not connected");
  } else
  {
    diag_ok("Connected");
  }
}
