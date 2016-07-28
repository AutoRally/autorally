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
 * @date January 29, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief ROS Interface Node for controlling servos
 *        Brief description continued.
 *
 * @details Detailed file description starts here.
 ***********************************************/

#include <boost/lexical_cast.hpp>
#include <pluginlib/class_list_macros.h>
#include "servoInterface.h"

PLUGINLIB_DECLARE_CLASS(autorally_core, ServoInterface, autorally_core::ServoInterface, nodelet::Nodelet)

namespace autorally_core
{

ServoInterface::~ServoInterface()
{}

void ServoInterface::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  std::string port;
  if(!nh.getParam(getName()+"/port", port))
  {
    NODELET_ERROR("ServoInterface: could not get servo interface port");
  }

  loadServoParams();
  loadServoCommandPriorities();

  m_maestro.init(nh, getName(), port);
  //m_ss.init(nh);

  double servoCommandRate = 0.0;
  if(!nhPvt.getParam("servoCommandRate", servoCommandRate) ||
     !nhPvt.getParam("throttleBrakeCoupled", m_brakeSetup.coupledWithThrottle) ||
     !nhPvt.getParam("servoCommandMaxAge", m_servoCommandMaxAge))
  {
    NODELET_ERROR_STREAM(getName() << " could not get all startup params");
  }

  for (auto& mapIt : m_servoCommandMsgs)
  {
    std::string commandTopic = mapIt.first+"/chassisCommand";
    ros::Subscriber sub = nh.subscribe(commandTopic, 1,
                        &ServoInterface::chassisCommandCallback, this);
    m_servoSub[mapIt.first] = sub;
    NODELET_INFO_STREAM("ServoInterface: subscribed to chassis command:" << commandTopic);
  }

  m_runstopMaxAge = ros::Duration(1.0);
  m_runstopSub = nh.subscribe("/runstop", 5, &ServoInterface::runstopCallback, this);  

  m_throttleTimer = nh.createTimer(ros::Rate(servoCommandRate),
                      &ServoInterface::setServos, this);

  m_chassisStatePub = nh.advertise<autorally_msgs::chassisState>
                     ("chassisState", 1);
}

void ServoInterface::chassisCommandCallback(
                     const autorally_msgs::chassisCommandConstPtr& msg)
{
  std::map<std::string, autorally_msgs::chassisCommand>::iterator mapIt;
  if((mapIt = m_servoCommandMsgs.find(msg->sender)) == m_servoCommandMsgs.end())
  {
    NODELET_ERROR_STREAM("ServoInterface: Unknown controller " <<
                         msg->sender <<
                         " attempting to control servos, please add entry " <<
                         " to servoCommandPriorities.yaml");
  } else
  {
    mapIt->second = *msg;

  }
}

void ServoInterface::setServos(const ros::TimerEvent&)
{

  autorally_msgs::chassisStatePtr chassisState(new autorally_msgs::chassisState);
  
  chassisState->steeringCommander = "";
  chassisState->steering = 0.0;

  chassisState->throttleCommander = "";
  chassisState->throttle = 0.0;

  chassisState->frontBrakeCommander = "";
  chassisState->frontBrake = 0.0;

  ros::Time currentTime = ros::Time::now();

  //check if motion is enabled (all runstop message runstopMotionEnabled = true)
  if(m_runstops.empty())
  {
    chassisState->runstopMotionEnabled = false;
  } else
  {
    chassisState->runstopMotionEnabled = true;
    for(auto& runstop : m_runstops)
    {
      if(currentTime-runstop.second.header.stamp < m_runstopMaxAge)
      {
        if(runstop.second.motionEnabled == 0)
        {
          chassisState->runstopMotionEnabled = false;
          chassisState->throttleCommander = "runstop";
        }
      }
    }
  }

  //find highest priority (lowest priority value) command message for each servo
  //across all currently valid servo commands
  std::vector<priorityEntry>::const_iterator vecIt;
  for(vecIt = m_servoCommandPriorities.begin();
      vecIt != m_servoCommandPriorities.end();
      vecIt++)
  {
    if( currentTime-m_servoCommandMsgs[vecIt->id].header.stamp <
        ros::Duration(m_servoCommandMaxAge))
    {
      if(chassisState->throttleCommander.empty() && chassisState->runstopMotionEnabled &&
         m_servoCommandMsgs[vecIt->id].throttle <= 1.0 &&
         m_servoCommandMsgs[vecIt->id].throttle >= -1.0)
      {
        chassisState->throttleCommander = m_servoCommandMsgs[vecIt->id].sender;
        chassisState->throttle = m_servoCommandMsgs[vecIt->id].throttle;
      }

      //valid steeringBrake commands are on [-1,1]
      if(chassisState->steeringCommander.empty() &&
         m_servoCommandMsgs[vecIt->id].steering <= 1.0 &&
         m_servoCommandMsgs[vecIt->id].steering >= -1.0)
      {
        chassisState->steeringCommander = m_servoCommandMsgs[vecIt->id].sender;
        chassisState->steering = m_servoCommandMsgs[vecIt->id].steering;
      }

      //valid frontBrake commands are on [0,1]
      if(chassisState->frontBrakeCommander.empty() &&
         m_servoCommandMsgs[vecIt->id].frontBrake <= 1.0 &&
         m_servoCommandMsgs[vecIt->id].frontBrake >= 0.0)
      {
        chassisState->frontBrakeCommander = m_servoCommandMsgs[vecIt->id].sender;
        chassisState->frontBrake = m_servoCommandMsgs[vecIt->id].frontBrake;
      }
    }
  }

  //only set servos if a valid command value was found
  if(!chassisState->throttleCommander.empty() && chassisState->runstopMotionEnabled == true)
  {
    setServo("throttle", chassisState->throttle);
    chassisState->throttle = chassisState->throttle;
  }

  if(!chassisState->steeringCommander.empty())
  {
    setServo("steering", chassisState->steering);
    chassisState->steering = chassisState->steering;
  } else
  {
    chassisState->steering = -10.0;
  }

  if(!chassisState->frontBrakeCommander.empty())
  {
    setServo("frontBrake", std::max(chassisState->frontBrake, 0.0));
    chassisState->frontBrake = std::max(chassisState->frontBrake, 0.0);
  } else
  {
    chassisState->frontBrake = -10.0;
  }

  m_maestro.m_serialPort.diag("steering commander:", chassisState->steeringCommander);
  m_maestro.m_serialPort.diag("throttle commander:", chassisState->throttleCommander);
  if(chassisState->frontBrakeCommander.empty())
  {
    chassisState->frontBrakeCommander = "n/a";
  }
  m_maestro.m_serialPort.diag("frontBrake commander:", chassisState->frontBrakeCommander);
  
  chassisState->header.stamp = ros::Time::now();
  chassisState->header.frame_id = "servoController";
  m_chassisStatePub.publish(chassisState);
  m_maestro.m_serialPort.tick("chassisState");
}

bool ServoInterface::setServo(const std::string& channel, const double target)
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

bool ServoInterface::getServo(const std::string& channel, double& position)
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

void ServoInterface::loadServoParams()
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
        NODELET_ERROR("ServoInterface: XmlRpc servo settings formatted incorrectly");
      }
    }
  } else
  {
    NODELET_ERROR("ServoInterface: Couldn't retreive servo settings");
  }
  NODELET_INFO("ServoInterface: Loaded %lu servos", m_servoSettings.size());

  if(m_servoSettings.find("frontBrake") != m_servoSettings.end())
  {
    m_brakeSetup.independentFront = true;
  }
  if(m_servoSettings.find("backBrake") != m_servoSettings.end())
  {
    m_brakeSetup.independentBack = true;
  }
}

void ServoInterface::loadServoCommandPriorities()
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
      m_servoCommandMsgs[mapIt->first] = autorally_msgs::chassisCommand();

    } else
    {
      NODELET_ERROR("ServoInterface: XmlRpc servo command priorities formatted incorrectly");
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
    NODELET_INFO_STREAM("ServoInterface: ServoCommand ID:Priorities:" << vecIt->id << ":" << vecIt->priority);
  }
  NODELET_INFO_STREAM("ServoInterface: Loaded " <<
                      m_servoCommandPriorities.size() << " servo commanders");
}

}
