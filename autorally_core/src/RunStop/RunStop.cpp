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
 * @file RunStop.cpp
 * @author Alex Bettadapur <alexbettadapur@gmail.com>
 * @date January 22, 2013
 * @copyright 2013 Georgia Institute of Technology
 * @brief Publishes a safeSpeed message based on run stop state
 *
 * @details Contains RunStop class implementation
 ***********************************************/
#include <sstream>
#include "RunStop.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RunStop");
	ros::NodeHandle nh;
  double safeSpeedDuration = 0.1;
  double safeSpeedMax = 0.0;
  double safeSpeedCaution = 0.0;

	std::string runStopPort;
	if(!nh.getParam("runStop/port", runStopPort) ||
	   !nh.getParam("safeSpeedMax", safeSpeedMax)||
	   !nh.getParam("safeSpeedCaution", safeSpeedCaution))
	{
		ROS_ERROR("Could not get all RunStop parameters");
		return -1;
	}
	nh.param<double>("safeSpeedDuration", safeSpeedDuration, 0.1);

	RunStop runStop(nh, runStopPort, safeSpeedMax, safeSpeedCaution);
	runStop.m_safeSpeedPub = nh.advertise<autorally_msgs::safeSpeed>("safeSpeed", 5);

	runStop.m_doWorkTimer = nh.createTimer(ros::Duration(safeSpeedDuration),
	                                     &RunStop::doWorkTimerCallback,
	                                     &runStop);

	ros::spin();
	return 0;
}

RunStop::RunStop(ros::NodeHandle &nh,
                             const std::string& port,
                             double safeSpeedMax,
                             double safeSpeedCaution):
	SerialSensorInterface(nh, "","RunStop", port, true),
	m_safeSpeedMax(safeSpeedMax),
	m_safeSpeedCaution(safeSpeedCaution),
  m_state("RED")
{
 	m_safeSpeedData.header.seq=0;
  m_safeSpeedData.header.frame_id="RUNSTOP";
  m_safeSpeedData.sender = "RUNSTOP";

  m_lastMessageTime = ros::Time::now() + ros::Duration(1);
}

RunStop::~RunStop()
{}

//leftfront, rightfront, leftback, rightback, servo, camera
bool RunStop::processData()
{
  //std::cout << "m_data:" << m_data.c_str() << std::endl;
  //frame data, and parse if an entire message is waiting
  int startPosition = m_data.find("#");
  if(startPosition > 0)
    {
      m_data.erase(0, startPosition);
    }
  startPosition = m_data.find("#");

  size_t endPosition = m_data.find("\r\n");
  //std::cout << startPosition << " " << endPosition << std::endl;
  if(startPosition == 0 && endPosition!=std::string::npos)
  {
    m_lastMessageTime = ros::Time::now();
    std::string message = m_data.substr(0,endPosition);

    //std::cout << "message:" << message.c_str() << std::endl;
    int statusStart = message.find(":");
    m_state = message.substr(statusStart+1,std::string::npos);

    m_data.erase(0, endPosition+1);
    return true;
  } else
  {
    if(startPosition > 0)
    {
      m_data.erase(0, startPosition);
    }
    return false;
  }
}

void RunStop::doWorkTimerCallback(const ros::TimerEvent&)
{
  //get out all the messages
 	while(processData())
 	{
 	  //std::cout << "." << std::endl;
 	}
  //std::cout << "--state--" << m_state.size() << m_state.c_str() << "--" << std::endl;

//  std::cout << "m_state:" << m_state << "euhwfuib" << std::endl;

  m_safeSpeedData.header.stamp = ros::Time::now();
  if(m_state == "GREEN")
  {
    m_safeSpeedData.speed = m_safeSpeedMax;
  } else if(m_state == "YELLOW")
  {
    m_safeSpeedData.speed = m_safeSpeedCaution;
  } else
  {
    m_safeSpeedData.speed = 0.0;
  }

  //if no recent message, safespeed=0
 	if((ros::Time::now()-m_lastMessageTime).toSec() > 1.0)
 	{
 	  diag_error("No recent data from RUNSTOP");
 	  m_safeSpeedData.speed = 0.0;
 	}

  m_safeSpeedPub.publish(m_safeSpeedData);
 	tick("RUNSTOP Status");


}
