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
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date July 20, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief Publishes a autorally_msgs::runstop message based on run stop state
 *
 * @details Contains RunStop class implementation
 ***********************************************/
#include <sstream>
#include "RunStop.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RunStop");
	ros::NodeHandle nh;

	std::string runStopPort;
	if(!nh.getParam("runStop/port", runStopPort))
	{
		ROS_ERROR("Could not get all RunStop parameters");
		return -1;
	}

	RunStop runStop(nh, runStopPort);
	runStop.runstopPub_ = nh.advertise<autorally_msgs::runstop>("runstop", 1);

  //publish runsto pat 10Hz into system
	runStop.doWorkTimer_ = nh.createTimer(ros::Rate(10),
	                                     &RunStop::doWorkTimerCallback,
	                                     &runStop);

	ros::spin();
	return 0;
}

RunStop::RunStop(ros::NodeHandle &nh,
                             const std::string& port):
  state_("RED")
{
  serialPort_.init(nh, ros::this_node::getName(), "","RunStop", port, true);
  runstopData_.header.frame_id="RUNSTOP";
  runstopData_.sender = "RUNSTOP";

  lastMessageTime_ = ros::Time::now() + ros::Duration(1);
}

RunStop::~RunStop()
{}

//leftfront, rightfront, leftback, rightback, servo, camera
bool RunStop::processData()
{
  //frame data, and parse if an entire message is waiting
  int startPosition = serialPort_.m_data.find("#");
  if(startPosition > 0)
  {
    serialPort_.m_data.erase(0, startPosition);
  }
  startPosition = serialPort_.m_data.find("#");

  size_t endPosition = serialPort_.m_data.find("\r\n");
  //std::cout << startPosition << " " << endPosition << std::endl;
  if(startPosition == 0 && endPosition!=std::string::npos)
  {
    lastMessageTime_ = ros::Time::now();
    std::string message = serialPort_.m_data.substr(0,endPosition);

    //std::cout << "message:" << message.c_str() << std::endl;
    int statusStart = message.find(":");
    state_ = message.substr(statusStart+1,std::string::npos);

    serialPort_.m_data.erase(0, endPosition+1);
    return true;
  } else
  {
    if(startPosition > 0)
    {
      serialPort_.m_data.erase(0, startPosition);
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

  runstopData_.header.stamp = ros::Time::now();
  if(state_ == "GREEN")
  {
    runstopData_.motionEnabled = true;
  } else if(state_ == "YELLOW")
  {
    runstopData_.motionEnabled = false;
  } else
  {
    runstopData_.motionEnabled = false;
  }

  //if no recent message, runstop is false
 	if((ros::Time::now()-lastMessageTime_).toSec() > 1.0)
 	{
 	  serialPort_.diag_error("No recent data from runstop box");
 	  runstopData_.motionEnabled = false;
 	}

  runstopPub_.publish(runstopData_);
 	serialPort_.tick("runstop Status");
}
