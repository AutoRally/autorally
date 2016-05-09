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
  m_throttleDamping(0.0),
  m_steeringDamping(0.0),
  m_throttleEnabled(true),
  m_steeringEnabled(true)
{
  m_joySub = m_nh.subscribe("joy", 1, &JoystickControl::joyCallback, this);

  m_commandPub = m_nh.advertise
                 <autorally_msgs::servoMSG>
                 ("joystick/servoCommand", 5);
  m_safeSpeedPub = m_nh.advertise
                 <autorally_msgs::safeSpeed>
                 ("safeSpeed", 1);
  m_servoCommand = autorally_msgs::servoMSGPtr(new autorally_msgs::servoMSG);
  m_servoCommand->header.frame_id = "joystick";
  m_safeSpeed = autorally_msgs::safeSpeedPtr(new autorally_msgs::safeSpeed);
  m_safeSpeed->sender = "joystick";
  m_safeSpeed->speed = 0.0;

  if(!m_nh.getParam("joystickController/throttleDamping", m_throttleDamping) ||
     !m_nh.getParam("joystickController/steeringDamping", m_steeringDamping) || 
     !m_nh.getParam("joystickController/throttleAxis", m_throttleAxis) ||
     !m_nh.getParam("joystickController/steeringAxis", m_steeringAxis) ||
     !m_nh.getParam("joystickController/safeSpeedIncButton", m_safeSpeedIncButton) ||
     !m_nh.getParam("joystickController/safeSpeedDecButton", m_safeSpeedDecButton) ||
     !m_nh.getParam("joystickController/safeSpeedZeroButton1", m_safeSpeedZeroButton1) ||
     !m_nh.getParam("joystickController/safeSpeedZeroButton2", m_safeSpeedZeroButton2) ||
     !m_nh.getParam("joystickController/throttleEnableButton", m_throttleEnableButton) ||
     !m_nh.getParam("joystickController/steeringEnableButton", m_steeringEnableButton) ||
     !m_nh.getParam("joystickController/throttleBrakePaired", m_throttleBrakePaired) ||
     !m_nh.getParam("joystickController/frontBrake", m_frontBrake) ||
     !m_nh.getParam("joystickController/brakeAxis", m_brakeAxis) )
  {
    ROS_ERROR_STREAM("Couldn't get joystick control parameters");
  }

  m_safeSpeedTimer = m_nh.createTimer(ros::Rate(5),
                                    &JoystickControl::safeSpeedCallback,
                                    this);

}

JoystickControl::~JoystickControl()
{}

void JoystickControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  /* axes[0] is left/right of left sitck, axes[3] is up/down of right stick
   * I scale the range of values from the joystick [-1,1] to the range of
   * valid servo command values [0-254], and reverse the steering command so it
   * is drives as expected.
   */

  if(joy->buttons[m_safeSpeedIncButton] == 1 && m_safeSpeed->speed > 0.0)
  {
    m_safeSpeed->speed -= 0.5;
  } else if(joy->buttons[m_safeSpeedDecButton] == 1)
  {
    m_safeSpeed->speed += 0.5;
  } else if(joy->buttons[m_safeSpeedZeroButton1] == 1 || joy->buttons[m_safeSpeedZeroButton2] == 1)
  {
    m_safeSpeed->speed = 0.0;
  }

  //can enable/disable throttle control with left bumpers on game pad
  if(joy->buttons[m_throttleEnableButton] == 1)
  {
    m_throttleEnabled = !m_throttleEnabled;
  }
  
  //can enable/disable steering control with right bumpers on game pad
  if(joy->buttons[m_steeringEnableButton] == 1)
  {
    m_steeringEnabled = !m_steeringEnabled;
  }
  
  if(m_steeringEnabled)
  {
    m_servoCommand->steering = -m_steeringDamping*joy->axes[m_steeringAxis];
  } else
  {
    m_servoCommand->steering = -10.0;
  }
  
  if(m_throttleEnabled)
  {
    
    //if m_throttleBrakePaired, then throttle can be [-1,1],
    //otherwise require throttle to be in [0, 1]
    if(m_throttleBrakePaired)
    {
      m_servoCommand->throttle = m_throttleDamping*joy->axes[m_throttleAxis];
    }
    else
    {
      m_servoCommand->throttle = std::max((double)(m_throttleDamping*joy->axes[m_throttleAxis]), 0.0);
    }

    if(m_frontBrake && m_servoCommand->throttle < 0.0)
    {
      m_servoCommand->frontBrake = abs(m_servoCommand->throttle);
    } else
    {
      m_servoCommand->frontBrake = 0.0;
    }
  } else
  {
    m_servoCommand->throttle = -10.0;
    m_servoCommand->frontBrake = -10.0;
  }
  
  m_servoCommand->header.stamp = ros::Time::now();
  m_commandPub.publish(m_servoCommand);
}

void JoystickControl::safeSpeedCallback(const ros::TimerEvent& time)
{
  m_safeSpeed->header.stamp = ros::Time::now();
  m_safeSpeedPub.publish(m_safeSpeed);
}
