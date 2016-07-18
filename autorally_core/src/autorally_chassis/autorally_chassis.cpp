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
 * @file autorally_chassis.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 29, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief ROS Interface Node for controlling servos
 *        Brief description continued.
 *
 * @details Detailed file description starts here.
 ***********************************************/

#include <boost/lexical_cast.hpp>
#include <pluginlib/class_list_macros.h>
#include "autorally_chassis.h"

PLUGINLIB_DECLARE_CLASS(autorally_core, autorally_chassis, autorally_core::autorally_chassis, nodelet::Nodelet)

namespace autorally_core
{

autorally_chassis::~autorally_chassis()
{}

void autorally_chassis::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  std::string port;
  if(!nh.getParam(getName()+"/port", port))
  {
    NODELET_ERROR("autorally_chassis: could not get servo interface port");
  }

  loadServoParams();
  loadServoCommandPriorities();

  m_maestro.init(nh, getName(), port);
  m_ss.init(nh);

  double servoCommandRate = 0.0;
  if(!nhPvt.getParam("servoCommandRate", servoCommandRate) ||
     !nhPvt.getParam("throttleBrakeCoupled", m_brakeSetup.coupledWithThrottle) ||
     !nhPvt.getParam("servoCommandMaxAge", m_servoCommandMaxAge))
  {
    NODELET_ERROR_STREAM(getName() << " could not get all startup params");
  }

  for (auto& mapIt : m_servoCommandMsgs)
  {
    ros::Subscriber sub = nh.subscribe(mapIt.first+"/servoCommand", 2,
                        &autorally_chassis::servoMSGCallback, this);
    m_servoSub[mapIt.first] = sub;
    NODELET_INFO_STREAM("autorally_chassis: subscribed to srv cmd:" << mapIt.first+"/servoCommand");
  }
  
  m_speedCommandSub = nh.subscribe("vehicleSpeedCommand", 2,
                        &autorally_chassis::speedCallback, this);

  //m_servoStatusTimer = nh.createTimer(ros::Rate(servoStatusPubRate),
  //                    &autorally_chassis::servoStatusTimerCallback, this);
  m_throttleTimer = nh.createTimer(ros::Rate(servoCommandRate),
                      &autorally_chassis::setServos, this);

  m_servoMSGPub = nh.advertise<autorally_msgs::servoMSG>
                     ("servoStatus", 2);

  //m_servoMSGStatus = autorally_msgs::servoMSGPtr(new autorally_msgs::servoMSG);
  //m_servoMSGStatus->header.frame_id = "servoController";
}

void autorally_chassis::servoMSGCallback(
                     const autorally_msgs::servoMSGConstPtr& msg)
{
  std::map<std::string, autorally_msgs::servoMSG>::iterator mapIt;
  if((mapIt = m_servoCommandMsgs.find(msg->header.frame_id)) == m_servoCommandMsgs.end())
  {
    NODELET_ERROR_STREAM("autorally_chassis: Unknown controller " <<
                         msg->header.frame_id <<
                         " attempting to control servos, please add entry " <<
                         " to servoCommandPriorities.yaml");
  } else
  {
    mapIt->second = *msg;

  }
}

void autorally_chassis::setServos(const ros::TimerEvent&)
{
  bool throttleFound = false;
  bool frontBrakeFound = false;
  bool backBrakeFound = false;
  bool steeringFound = false;
  double throttle = -10.0;
  double frontBrake = -10.0;
  double backBrake = -10.0;
  double steering = -10.0;

  //find highest priority (lowest priority value) command message for each servo
  //across all currently valid servo commands
  ros::Time currentTime = ros::Time::now();

  std::vector<priorityEntry>::const_iterator vecIt;
  for(vecIt = m_servoCommandPriorities.begin();
      vecIt != m_servoCommandPriorities.end();
      vecIt++)
  {
    if( currentTime-m_servoCommandMsgs[vecIt->id].header.stamp <
        ros::Duration(m_servoCommandMaxAge))
    {
      if(!throttleFound &&
         m_servoCommandMsgs[vecIt->id].throttle <= 1.0 &&
         m_servoCommandMsgs[vecIt->id].throttle >= -1.0)
      {
        throttleFound = true;
        throttle = m_servoCommandMsgs[vecIt->id].throttle;
      }

      if(!steeringFound &&
         m_servoCommandMsgs[vecIt->id].steering <= 1.0 &&
         m_servoCommandMsgs[vecIt->id].steering >= -1.0)
      {
        steeringFound = true;
        steering = m_servoCommandMsgs[vecIt->id].steering;
      }

      if(!frontBrakeFound &&
         m_servoCommandMsgs[vecIt->id].frontBrake <= 1.0 &&
         m_servoCommandMsgs[vecIt->id].frontBrake >= -1.0)
      {
        frontBrakeFound = true;
        frontBrake = m_servoCommandMsgs[vecIt->id].frontBrake;
      }

      if(!backBrakeFound &&
         m_servoCommandMsgs[vecIt->id].backBrake <= 1.0 &&
         m_servoCommandMsgs[vecIt->id].backBrake >= -1.0)
      {
        backBrakeFound = true;
        backBrake = m_servoCommandMsgs[vecIt->id].backBrake;
      }
    }
  }

  autorally_msgs::servoMSGPtr servoStatus(new autorally_msgs::servoMSG);

  //only set servos if a valid command value was found
  if(throttleFound)
  {
    double safeThrottle = m_ss.safeThrottle(throttle);
    //NODELET_INFO_STREAM("safeThrottle:" << safeThrottle << " throttle:" << throttle);
    if(safeThrottle != throttle)
    {
      m_maestro.m_serialPort.diag("in control", "safeThrottle");
    } else
    {
      m_maestro.m_serialPort.diag("in control", "servoCommand");
    }

    //if theres no brake with the throttle, throttle cant go negative
    if(!m_brakeSetup.coupledWithThrottle)
    {
      safeThrottle = std::max(safeThrottle, 0.0);
    }
    setServo("throttle", safeThrottle);
    servoStatus->throttle = safeThrottle;
  }

  if(steeringFound)
  {
    setServo("steering", steering);
    servoStatus->steering = steering;
  } else
  {
    servoStatus->steering = -10.0;
  }

  if(frontBrakeFound && m_brakeSetup.independentFront)
  {
    setServo("frontBrake", std::max(frontBrake, 0.0));
    servoStatus->frontBrake = std::max(frontBrake, 0.0);
  } else
  {
    servoStatus->frontBrake = -10.0;
  }
  if(backBrakeFound && m_brakeSetup.independentBack)
  {
    setServo("backBrake", std::max(backBrake, 0.0));
    servoStatus->backBrake = std::max(backBrake, 0.0);
  } else
  {
    servoStatus->backBrake = -10.0;
  }

  servoStatus->header.stamp = ros::Time::now();
  servoStatus->header.frame_id = "servoController";
  m_servoMSGPub.publish(servoStatus);
  m_maestro.m_serialPort.tick("servoStatus");
}

bool autorally_chassis::setServo(const std::string& channel, const double target)
{
  if(target > 1.0 || target < -1.0)
  {
    return false;
    NODELET_WARN_STREAM("Servo value " << target <<
                        " for channel " << channel <<
                        " out of [-1,1] range");
  }

  std::map<std::string, ServoSettings>::const_iterator mapIt;
  double pos = target;
  if( (mapIt = m_servoSettings.find(channel)) != m_servoSettings.end())
  {
    if(mapIt->second.reverse)
    {
      pos = -pos;
    }

    if(pos > 0.0)
    {
      pos = mapIt->second.center+pos*(mapIt->second.max-mapIt->second.center);
    } else if(pos < 0.0)
    {
      pos = mapIt->second.center+pos*(mapIt->second.center-mapIt->second.min);
    } else
    {
      pos = mapIt->second.center;
    }
    unsigned int val = static_cast<unsigned int>(pos*4);
    m_maestro.setTargetMS(mapIt->second.port, val);
    return true;
  }

//  m_maestro.diag_error("Servo does not exist:"+channel);
  return false;
}

bool autorally_chassis::getServo(const std::string& channel, double& position)
{
  std::map<std::string, ServoSettings>::const_iterator mapIt;
  unsigned short val;
  if( (mapIt = m_servoSettings.find(channel)) != m_servoSettings.end())
  {
    val = m_maestro.getTarget(mapIt->second.port);
    position = (double)val/4.0;

    if(position > mapIt->second.center)
    {
      position = (position-mapIt->second.center)/(mapIt->second.max-mapIt->second.center);
    } else if(position < mapIt->second.center)
    {
      position = (position-mapIt->second.center)/(mapIt->second.center-mapIt->second.min);
    } else
    {
      position = 0.0;
    }

    if(mapIt->second.reverse)
    {
      position = -position;
    }
//    std::cout << "got channel (" << (int)mapIt->second.port << ") val:" << val << " pos:" << position << std::endl;
    return true;
    //TODO: handle reverse
  }
  m_maestro.m_serialPort.diag_error("Servo does not exist:"+channel);
  return false;
}

/*
void autorally_chassis::servoStatusTimerCallback(const ros::TimerEvent&)
{
  populateStatusMessage(m_servoMSGStatus);
  m_servoMSGStatus->header.stamp = ros::Time::now();
  m_servoMSGPub.publish(m_servoMSGStatus);
  m_maestro.m_serialPort.tick("servoStatus");
}

void autorally_chassis::populateStatusMessage(autorally_msgs::servoMSGPtr& status)
{
  if( (status->safeSpeed = m_ss.getSafeSpeed()) == -1.0)
  {
    NODELET_ERROR("No valid safeSpeed");
    m_maestro.m_serialPort.diag_error("NO valid safeSpeed");
  }

  double position;
  if(getServo("steering", position))
  {
    status->steering = position;
  }
  if(getServo("throttle", position))
  {
    status->throttle = position;
  }
  if(getServo("frontBrake", position))
  {
    status->frontBrake = position;
  }
  if(getServo("backBrake", position))
  {
    status->backBrake = position;
  }
}
*/

void autorally_chassis::loadServoParams()
{
  //read in servo settings
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  XmlRpc::XmlRpcValue v;
  nhPvt.param("servos", v, v);
  if(v.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    XmlRpc::XmlRpcValue servoInfo;
    ServoSettings toAdd;
    for(int i = 0; i < v.size(); i++)
    {
      servoInfo = v[boost::lexical_cast<std::string>(i)];

      if(servoInfo.getType() == XmlRpc::XmlRpcValue::TypeStruct &&
         servoInfo.size() == 5)
      {
        toAdd.center = static_cast<int>(servoInfo["center"]);
        toAdd.min = static_cast<int>(servoInfo["min"]);
        toAdd.max = static_cast<int>(servoInfo["max"]);
        toAdd.range = toAdd.max-toAdd.min;
        toAdd.port = i;
        toAdd.reverse = static_cast<bool>(servoInfo["reverse"]);
        m_servoSettings[servoInfo["name"]] = toAdd;
      } else
      {
        NODELET_ERROR("autorally_chassis: XmlRpc servo settings formatted incorrectly");
      }
    }
  } else
  {
    NODELET_ERROR("autorally_chassis: Couldn't retreive servo settings");
  }
  NODELET_INFO("autorally_chassis: Loaded %lu servos", m_servoSettings.size());

  if(m_servoSettings.find("frontBrake") != m_servoSettings.end())
  {
    m_brakeSetup.independentFront = true;
  }
  if(m_servoSettings.find("backBrake") != m_servoSettings.end())
  {
    m_brakeSetup.independentBack = true;
  }
}

void autorally_chassis::loadServoCommandPriorities()
{
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  XmlRpc::XmlRpcValue v;
  nhPvt.param("servoCommandProirities", v, v);
  std::map<std::string, XmlRpc::XmlRpcValue>::iterator mapIt;
  for(mapIt = v.begin(); mapIt != v.end(); mapIt++)
  {
    if(mapIt->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      //add entry in priority queue and command map
      priorityEntry toAdd;
      toAdd.id = mapIt->first;
      toAdd.priority = static_cast<int>(mapIt->second);
      m_servoCommandPriorities.push_back(toAdd);
      m_servoCommandMsgs[mapIt->first] = autorally_msgs::servoMSG();

    } else
    {
      NODELET_ERROR("autorally_chassis: XmlRpc servo command priorities formatted incorrectly");
    }
  }
  std::sort(m_servoCommandPriorities.begin(),
            m_servoCommandPriorities.end(),
            priorityComparator());

  std::vector<priorityEntry>::const_iterator vecIt;
  for(vecIt = m_servoCommandPriorities.begin();
      vecIt != m_servoCommandPriorities.end();
      vecIt++)
  {
    NODELET_INFO_STREAM("autorally_chassis: ServoCommand ID:Priorities:" << vecIt->id << ":" << vecIt->priority);
  }
  NODELET_INFO_STREAM("autorally_chassis: Loaded " <<
                      m_servoCommandPriorities.size() << " servo commanders");
}

}
