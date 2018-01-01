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
 * @file hpiController.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 19, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief Brief file description
 *        Brief description continued.
 *
 * @details Detailed file description starts here.
 ***********************************************/

#include "JoystickControl.h"

JoystickControl::JoystickControl():
  throttleDamping_(0.0),
  steeringDamping_(0.0),
  throttleEnabled_(true),
  steeringEnabled_(true)
{
  m_joySub = nh_.subscribe("joy", 1, &JoystickControl::joyCallback, this);

  runstopPub_ = nh_.advertise
                 <autorally_msgs::runstop>
                 ("runstop", 1);

  commandPub_ = nh_.advertise
                 <autorally_msgs::chassisCommand>
                 ("joystick/chassisCommand", 1);

  runstop_.sender = "joystick";
  runstop_.motionEnabled = false;


  if(!nh_.getParam("joystickController/throttleDamping", throttleDamping_) ||
     !nh_.getParam("joystickController/steeringDamping", steeringDamping_) ||
     !nh_.getParam("joystickController/throttleAxis", throttleAxis_) ||
     !nh_.getParam("joystickController/steeringAxis", steeringAxis_) ||
     !nh_.getParam("joystickController/throttleEnableButton", throttleEnableButton_) ||
     !nh_.getParam("joystickController/steeringEnableButton", steeringEnableButton_) ||
     !nh_.getParam("joystickController/brakeAxis", brakeAxis_) )
  {
    ROS_ERROR_STREAM("Couldn't get joystick control parameters");
  }

  XmlRpc::XmlRpcValue v;
  nh_.param("joystickController/runstopToggleButtons", v, v);
  for(int i =0; i < v.size(); i++)
  {
    runstopToggleButtons_.push_back(v[i]);
  }

  runstopTimer_ = nh_.createTimer(ros::Rate(5),
                                   &JoystickControl::runstopCallback,
                                   this);

}

JoystickControl::~JoystickControl()
{}

void JoystickControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  /* axes[0] is left/right of left sitck, axes[3] is up/down of right stick
   * I scale the range of values from the joystick [-1,1] to the range of
   * valid command values [-1.0, 1.0], and reverse the steering command so it
   * is drives as expected.
   */

  //toggle runstop if a runstop toggle button changed from 0 to 1
  for(auto vecIt : runstopToggleButtons_)
  {
    if(joy->buttons[vecIt] == 1 && prevJoy_.buttons[vecIt] == 0)
    {
      runstop_.motionEnabled = !runstop_.motionEnabled;
    }
  }

  //can enable/disable throttle control with L2 on game pad, only toggle if button changed from 0 to 1
  if(joy->buttons[throttleEnableButton_] == 1 && prevJoy_.buttons[throttleEnableButton_] == 0)
  {
    throttleEnabled_ = !throttleEnabled_;
  }

  //can enable/disable steering control with R2 on game pad, only toggle if button changed from 0 to 1
  if(joy->buttons[steeringEnableButton_] == 1 && prevJoy_.buttons[steeringEnableButton_] == 0)
  {
    steeringEnabled_ = !steeringEnabled_;
  }

  if(steeringEnabled_)
  {
    chassis_command_.steering = -steeringDamping_*joy->axes[steeringAxis_];
  } else
  {
    chassis_command_.steering = -10.0;
  }

  if(throttleEnabled_)
  {

    chassis_command_.throttle = throttleDamping_*joy->axes[throttleAxis_];

    if(chassis_command_.throttle < 0.0)
    {
      chassis_command_.frontBrake = fabs(chassis_command_.throttle);
    } else
    {
      chassis_command_.frontBrake = 0.0;
    }
  } else
  {
    chassis_command_.throttle = -10.0;
    chassis_command_.frontBrake = -10.0;
  }

  prevJoy_ = *joy;
  chassis_command_.header.frame_id = "joystick";
  chassis_command_.sender = "joystick";
  chassis_command_.header.stamp = ros::Time::now();
  commandPub_.publish(chassis_command_);
}

void JoystickControl::runstopCallback(const ros::TimerEvent& time)
{
  runstop_.header.stamp = ros::Time::now();
  runstopPub_.publish(runstop_);
}
