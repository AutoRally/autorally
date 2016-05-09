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
 * @file SerialSensorInterface.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief SerialSensorInterface class implementation
 *
 ***********************************************/
#include <autorally_core/SerialSensorInterface.h>

#include <ros/time.h>

SerialSensorInterface::SerialSensorInterface() :
  m_port(""),
  m_settingsApplied(false)
{}

SerialSensorInterface::SerialSensorInterface(ros::NodeHandle &nh,
                                             const std::string portHandle,
                                             const std::string hardwareID,
                                             const std::string port,
                                             const bool queueData) :
  SerialCommon(portHandle, hardwareID, port),
  m_port(port),
  m_settingsApplied(false),
  m_queueData(queueData),
  m_mostRecentData(ros::Time::now())
{
  init(nh, ros::this_node::getName(), portHandle, hardwareID, port, queueData);
}



SerialSensorInterface::~SerialSensorInterface()
{
  /*
  ** std::cout is used here instead of ROS_INFO because
  ** when the destructor is called, ros::shutdown has already been executed
  ** and roscore is not running. So ROS_INFO will not print any output.
  ** Additionally, in the launch file the output param must be set as "screen"
  ** to be able to view the std::cout outputs.
  */
  //std::cout << "Shutting down " << m_port.c_str() << " " << close(fileDescriptor()) << std::endl;
}

void SerialSensorInterface::init(ros::NodeHandle &nh,
                                 const std::string nodeName,
                                 const std::string portHandle,
                                 const std::string hardwareID,
                                 const std::string port,
                                 const bool queueData)
{
  std::string parity;
  int baud, stopBits, dataBits;
  bool hardwareFlow, softwareFlow;
  m_port = port;
  m_queueData = queueData;

  SerialCommon::init(portHandle, hardwareID, m_port);

  //get current node name to allow access to the serial parameters
  //specific to this node
  std::string nName = nodeName+((portHandle.length()>0)?"/"+portHandle:"");
  if(!nh.getParam(nName+"/serialBaud", baud) ||
     !nh.getParam(nName+"/serialParity", parity) ||
     !nh.getParam(nName+"/serialStopBits", stopBits) ||
     !nh.getParam(nName+"/serialDataBits", dataBits) ||
     !nh.getParam(nName+"/serialHardwareFlow", hardwareFlow) ||
     !nh.getParam(nName+"/serialSoftwareFlow", softwareFlow))
  {
    ROS_ERROR("Could not get all serialSensorInterface parameters for %s", nName.c_str());
  }

  m_settingsApplied = connect(m_port,
                              baud,
                              parity,
                              stopBits,
                              dataBits,
                              hardwareFlow,
                              softwareFlow);

  if(m_queueData && m_settingsApplied)
  {

    //set timer to be able to pull 25% more data off serial port than possible
    m_serialTimer = nh.createTimer(ros::Duration(1.0/((baud/100)*1.25)),
                      &SerialSensorInterface::pollSerial, this);
  }
}


std::string SerialSensorInterface::readPort()
{
  char data[100];
  //get up to 100 bytes from the com port, add it to the data buffer
  int received = 0;
  if(connected())
  {
    if( (received = read(fileDescriptor(), &data, 100)) >= 0)
    {
      //std::cout<<"true";
      data[received]='\0';
      if((int)data[received-1]==10)
      {
        data[received-1]='\0';
        received--;
      }
      m_data.append(std::string(data));
      m_mostRecentData = ros::Time::now();
      return std::string(data);
    }
  }
  return std::string();
}

void SerialSensorInterface::pollSerial(const ros::TimerEvent& /*time*/)
{
  char data[100];
  //get up to 100 bytes from the com port, add it to the data buffer
  int received = 0;
  //ROS_INFO("Polling");
  if(connected())
  {
    if( (received = read(fileDescriptor(), &data, 100)) >= 0)
    {
      //ROS_INFO("RECIEVED");
      //data[received]='\0';
      m_data.append(data, received);
      m_mostRecentData = ros::Time::now();
    }
  }
}

void SerialSensorInterface::diagnosticStatus(const ros::TimerEvent& /*time*/)
{
  //queue up a status messages
  if(!m_settingsApplied)
  {
    diag_error("Serial port setting error: "+getSettingError());
  }

  if(!connected())
  {
    diag_error("Not connected");
  } else if( m_queueData && (ros::Time::now() - m_mostRecentData).toSec() > 1.0)
  {
    //diag_warn("No data within previous second");
  } else
  {
    diag_ok("Connected");
  }
}
