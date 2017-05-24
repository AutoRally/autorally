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
 * @file status_monitor.cpp
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Implementation of the status monitor
 ***********************************************/

#include <autorally_control/path_integral/status_monitor.h>

namespace autorally_control {

StatusMonitor::StatusMonitor(ros::NodeHandle nh)
{
	last_status_ = ros::Time::now();
	status_sub_ = nh.subscribe("/mppi_controller/mppiStatus", 1, &StatusMonitor::statusCallback, this);
	std::string info = "MPPI Controller";
	std::string hardwareID = "none";
	std::string portPath = "";
	Diagnostics::init(info, hardwareID, portPath);
}

void StatusMonitor::statusCallback(autorally_msgs::pathIntegralStatus msg)
{
	info_ = msg.info;
	status_ = msg.status;
	last_status_ = ros::Time::now();
}

void StatusMonitor::diagnosticStatus(const ros::TimerEvent& time)
{
	if ((double)ros::Time::now().toSec() - (double)last_status_.toSec() > TIMEOUT){
		diag_error("CONTROLLER TIMEOUT");
	}
	else if (status_ == 0){
		diag_ok(info_);
	}
	else if (status_ == 1){
		diag_warn(info_);
	}
	else if (status_ == 2){
		diag_error(info_);
	}
}

}

using namespace autorally_control;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mppiStatusMonitor");
	ros::NodeHandle status_node("~");
	StatusMonitor monitor(status_node);
	ros::spin();
}