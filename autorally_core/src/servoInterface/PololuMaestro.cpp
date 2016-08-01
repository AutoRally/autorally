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
 * @file PololuMaestro.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 19, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief Implementation of PololuMaestro class
 *
 ***********************************************/

#include <autorally_core/PololuMaestro.h>

PololuMaestro::PololuMaestro()
{}

PololuMaestro::~PololuMaestro()
{}

void PololuMaestro::init(ros::NodeHandle &nh,
                         const std::string& nodeName,
                         const std::string& port)
{
    m_serialPort.init(nh, nodeName, "", "PololuMaestro", port, false);
}

void PololuMaestro::setTarget(const unsigned char channel, const unsigned char target)
{
  unsigned char command[3];

  command[0] = 0xFF; // Command byte: Set Target.
  command[1] = channel; // First data byte holds channel number.
  command[2] = target; // Target position

  //ROS_INFO("Sending %d %d %d", command[0], command[1], command[2]);

  if(m_serialPort.writePort(command, 3) != 3)
  {
    //ROS_ERROR("Failed to set state");
    m_serialPort.diag_error("Failed to set state");
  }
}

void PololuMaestro::setTargetMS(const unsigned char channel, const unsigned int target)
{
  unsigned char command[4];

  command[0] = 0x84; // Command byte: Set Target.
  command[1] = channel; // First data byte holds channel number.
  command[2] = target&0x7F; // Target position low
  command[3] = (target>>7) & 0x7F; // Target position high

  //ROS_INFO("Sending %d %d %d", command[0], command[1], command[2]);

  if(m_serialPort.writePort(command, 4) != 4)
  {
    //ROS_ERROR("Failed to set state");
    m_serialPort.diag_error("Failed to set state");
  }
}

unsigned short PololuMaestro::getTarget(unsigned char channel)
{
  unsigned char command[2];
  unsigned char reply[2];

  command[0] = 0x90; // Command byte: Get Position.
  command[1] = channel; // First data byte holds channel number.

  if(m_serialPort.writePort(command, 2) != 2)
  {
    //ROS_ERROR("Failed to request state");
    //perror("Pololu: Serial Write error");    
    m_serialPort.diag_error("Failed to request state");
    return 0;
  }

  usleep(10);
  int r = 0;
  if( (r = read(m_serialPort.fileDescriptor(), &reply, 2)) != 2)
  {
    m_serialPort.diag_error("Error getting channel");
    //ROS_ERROR("Error getting channel %d : %d", channel, r);
  }

  return reply[0] + (reply[1]<<8);
}

void PololuMaestro::getErrors()
{
  unsigned char command = 0xA1;
  unsigned char reply[2];
  for(int i = 0; i<2; i++)
  {
    reply[i] = 0;
  }

  if(m_serialPort.writePort(&command, 1) != 1)
  {
    ROS_ERROR("PololuMaestro::getErrors: Failed to get errors");
    return;
  }

  if( read(m_serialPort.fileDescriptor(), &reply, 2) != 2)
  {
     ROS_ERROR("Error reading errors");
     m_serialPort.diag_error("Error reading errors");
  }

  ROS_ERROR("Error message:%c %c", reply[0], reply[1]);
}
