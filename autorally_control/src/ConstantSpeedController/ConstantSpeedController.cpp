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
 * @file servoInterface.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date April 14,, 2014
 * @copyright 2014 Georgia Institute of Technology
 * @brief Controller to drive robot at constant speed
 *
 * @details Detailed file description starts here.
 ***********************************************/

#include <math.h>
#include <pluginlib/class_list_macros.h>

#include "ConstantSpeedController.h"

#define PI 3.14159265
#define DEGTORAD (PI/180)

PLUGINLIB_DECLARE_CLASS(control, ConstantSpeedController, control::ConstantSpeedController, nodelet::Nodelet)

namespace control
{

ConstantSpeedController::ConstantSpeedController():
  m_controllerState(DISABLED),
  m_constantSpeedPrevThrot(0.0),
  m_controlEnabled(true),
  m_frontWheelsSpeed(0.0),
  m_backWheelsSpeed(0.0),
  m_integralError(0.0),
  m_servoMSG(new autorally_msgs::servoMSG)
{}

ConstantSpeedController::~ConstantSpeedController()
{}

void ConstantSpeedController::onInit()
{
  NODELET_INFO("ConstantSpeedController initialization");
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nhPvt = getPrivateNodeHandle();

  loadThrottleCalibration();

  m_mostRecentSafeSpeed.speed = 0;

  m_safeSpeedSub = nh.subscribe("safeSpeed", 1,
                          &ConstantSpeedController::safeSpeedCallback, this);
  m_wheelSpeedsSub = nh.subscribe("wheelSpeeds", 1,
                          &ConstantSpeedController::wheelSpeedsCallback,
                          this);
  m_servoMSGPub = nh.advertise<autorally_msgs::servoMSG>
                     ("constantSpeedController/servoCommand", 1);

  if(!nhPvt.getParam("speedCommander", m_speedCommander) ||
     !nhPvt.getParam("accelerationRate", m_accelerationRate) ||
     !nhPvt.getParam("accelerationRate", m_accelerationRate) ||
     !nhPvt.getParam("KP", m_constantSpeedKP) ||
     !nhPvt.getParam("KD", m_constantSpeedKD) ||
     !nhPvt.getParam("KI", m_constantSpeedKI) ||
     !nhPvt.getParam("IMax", m_constantSpeedIMax))
  {
    NODELET_ERROR("Could not get all ConstantSpeedController params");
  }

  m_servoMSG->header.frame_id = "constantSpeedController";
  m_servoMSG->steering = -5.0;
  m_servoMSG->frontBrake = 0.0;
  m_servoMSG->backBrake = 0.0;

  //m_accelerationProfile = generateAccelerationProfile(100);

  m_controlTimer = nh.createTimer(ros::Rate(1),
                      &ConstantSpeedController::controlCallback, this);
  /*m_controlEnableTimer = nh.createTimer(ros::Rate(0.2),
                      &ConstantSpeedController::enableControlCallback, this);*/
  NODELET_INFO("ConstantSpeedController initialization complete");

}

void ConstantSpeedController::safeSpeedCallback(const autorally_msgs::safeSpeedConstPtr& msg)
{
  //if the safespeed is from sender who can command the constant speed controller
  //if(msg->sender == m_speedCommander)
  //{
  if (m_mostRecentSafeSpeed.speed != msg->speed)
    {
      NODELET_INFO("ConstantSpeedController: set safespeed to %f", msg->speed);
    }
  m_mostRecentSafeSpeed = *msg;
  //}
  //else
  //{
    //NODELET_WARN("ConstantSpeedController: safespeed from invalid source");
  //}
}

void ConstantSpeedController::wheelSpeedsCallback(const autorally_msgs::wheelSpeedsConstPtr& msg)
{
  m_frontWheelsSpeed = 0.5*(msg->lfSpeed + msg->rfSpeed);
  //m_backWheelsSpeed = 0.2*m_backWheelsSpeed + 0.4*(msg->lbSpeed + msg->rbSpeed);

  if (m_mostRecentSafeSpeed.speed > 0.1)
  {
    m_servoMSG->header.stamp = ros::Time::now();
    double p;
    if(m_throttleMappings.interpolateKey(m_mostRecentSafeSpeed.speed, p))
    {
      m_integralError += m_mostRecentSafeSpeed.speed - m_frontWheelsSpeed;
      if (m_integralError > (m_constantSpeedIMax / m_constantSpeedKI))
      {
        m_integralError = (m_constantSpeedIMax / m_constantSpeedKI);
      }

      if (m_integralError < -(m_constantSpeedIMax / m_constantSpeedKI))
      {
        m_integralError = -(m_constantSpeedIMax / m_constantSpeedKI);
      }

      m_servoMSG->throttle = p +
                 m_constantSpeedKP*(m_mostRecentSafeSpeed.speed - m_frontWheelsSpeed);
      m_servoMSG->throttle += m_constantSpeedKI * m_integralError;
      m_servoMSG->throttle = std::max(0.0, std::min(1.0, m_servoMSG->throttle));

      NODELET_INFO("interp %f, command %f",p, m_servoMSG->throttle);
      m_servoMSGPub.publish(m_servoMSG);
      m_constantSpeedPrevThrot = p;
    }
    else
    {
      NODELET_WARN("ConstantSpeedController could not interpolate speed %f", m_mostRecentSafeSpeed.speed);
      m_servoMSG->throttle = 0;
    }
  }
  else
  {
    m_servoMSG->throttle = 0;
    m_servoMSGPub.publish(m_servoMSG);
  }
}

/*void ConstantSpeedController::enableControlCallback(const ros::TimerEvent& time)
{
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  if(!nhPvt.getParam("controlEnabled", m_controlEnabled) )
  {
    NODELET_ERROR("Could not update ConstantSpeedController params");
  }
}*/

void ConstantSpeedController::controlCallback(const ros::TimerEvent& time)
{
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  nhPvt.getParam("KP", m_constantSpeedKP);
  nhPvt.getParam("KD", m_constantSpeedKD);
  nhPvt.getParam("KI", m_constantSpeedKI);
  nhPvt.getParam("IMax", m_constantSpeedIMax);

}

void ConstantSpeedController::loadThrottleCalibration()
{
  NODELET_INFO("Loading calibration");
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  XmlRpc::XmlRpcValue v;
  nhPvt.param("throttleCalibration", v, v);
  std::map<std::string, XmlRpc::XmlRpcValue>::iterator mapIt;
  for(mapIt = v.begin(); mapIt != v.end(); mapIt++)
  {
    if(mapIt->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      std::pair<double, double> toAdd(std::pair<double, double>(
                                      boost::lexical_cast<double>(mapIt->first),
                                      static_cast<double>(mapIt->second)));
      NODELET_INFO_STREAM("ConstantSpeedController added to add mapping " <<
                             toAdd.first << ":" << toAdd.second);
      if(!m_throttleMappings.update(toAdd))
      {
        NODELET_ERROR_STREAM("ConstantSpeedController Failed to add mapping " <<
                             toAdd.first << ":" << toAdd.second);
      }
    } else
    {
      NODELET_ERROR("ConstantSpeedController: XmlRpc throttle calibration formatted incorrectly");
    }
  }
  NODELET_INFO_STREAM("ConstantSpeedController: Loaded " <<
                      m_throttleMappings.size() <<
                      " throttle mappings");
}

}
