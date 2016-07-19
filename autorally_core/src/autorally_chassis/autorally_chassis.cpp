/*
* Software License Agreement (BSD License)
* Copyright (c) 2016, Georgia Institute of Technology
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
 * @date July 10, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief Interface for an AutoRally chassis
 *
 * @details Implementation of the autorally_chassis class that controls and receives state information from an AutoRally
 *          chassis
 ***********************************************/

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
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
    NODELET_ERROR("autorally_chassis: could not get chassis interface port");
  }

  loadChassisConfig();
  loadChassisCommandPriorities();
  serialPort_.init(nh, getName(), "", "AutoRallyChassis", port, false);
  
  double commandRate = 0.0;
  double chassisCommandMaxAge = 0.0;
  double runstopMaxAge = 0.0;

  if(!nhPvt.getParam("commandRate", commandRate) ||
     !nhPvt.getParam("commandMaxAge", chassisCommandMaxAge) ||
     !nhPvt.getParam("runstopMaxAge", runstopMaxAge))
  {
    NODELET_ERROR_STREAM(getName() << " could not get all startup params");
  }
  chassisCommandMaxAge_ = ros::Duration(chassisCommandMaxAge);
  runstopMaxAge_ = ros::Duration(runstopMaxAge);

  chassisStatePub_ = nh.advertise<autorally_msgs::chassisState>
                     ("chassisStatus", 1);
  wheelSpeedsPub_ = nh.advertise<autorally_msgs::wheelSpeeds>
                     ("wheelSpeeds", 1);
  chassisCommandPub_ = nh.advertise<autorally_msgs::wheelSpeeds>
                     ("RC/chassisCommand", 1);

  for (auto& mapIt : chassisCommands_)
  {
    std::string topic = mapIt.first+"/chassisCommand";
    ros::Subscriber sub = nh.subscribe(topic, 1,
                                       &autorally_chassis::chassisCommandCallback, this);
    chassisCommandSub_[mapIt.first] = sub;
    NODELET_INFO_STREAM("autorally_chassis: subscribed to chassis cmd:" << topic);
  }
  runstopSub_ = nh.subscribe("/runstop", 4, &autorally_chassis::runstopCallback, this);  
  
  serialPort_.registerDataCallback(boost::bind(&autorally_chassis::chassisFeedbackCallback, this));

  chassisControlTimer_ = nh.createTimer(ros::Rate(commandRate),
                      &autorally_chassis::setChassisActuators, this);
}


void autorally_chassis::chassisCommandCallback(
                     const autorally_msgs::chassisCommandConstPtr& msg)
{
  std::map<std::string, autorally_msgs::chassisCommand>::iterator mapIt;
  if((mapIt = chassisCommands_.find(msg->header.frame_id)) == chassisCommands_.end())
  {
    NODELET_ERROR_STREAM("autorally_chassis: Unknown controller " <<
                         msg->header.frame_id <<
                         " attempting to control chassis, please add entry " <<
                         " to chassisCommandPriorities.yaml");
  } else
  {
    mapIt->second = *msg;
  }
}

void autorally_chassis::chassisFeedbackCallback()
{
  serialPort_.lock();
  size_t startPosition = serialPort_.m_data.find_first_of('#');
  size_t endPosition = serialPort_.m_data.find_first_of('\n', startPosition);
  std::string data = "";

  //make sure we have start and end position
  if(startPosition !=  std::string::npos && endPosition != std::string::npos)
  {
    //frame data if nto framed
    if(startPosition != 0)
    {
      serialPort_.m_data.erase(0, startPosition);
    }

    startPosition = 0;
    endPosition = serialPort_.m_data.find_first_of('\n', startPosition);

    //cut out and erase message from queue
    data = serialPort_.m_data.substr(0, endPosition);
    serialPort_.m_data.erase(0, endPosition);

  } else if(startPosition != std::string::npos)
  {
    serialPort_.m_data.erase(0, startPosition);
  }
  serialPort_.unlock();

  if(!data.empty())
  {
    processChassisMessage(data.substr(1,1), data.substr(2, endPosition-3));
  }
}

void autorally_chassis::processChassisMessage(std::string msgType, std::string msg)
{
  switch(msgType[0])
  {
    //wheel speeds data
    case 'w':
    {
      std::vector<std::string> data;
      boost::split(data, msg, boost::is_any_of(","));
      if(data.size() == 4)
      {
        autorally_msgs::wheelSpeedsPtr wheelSpeeds(new autorally_msgs::wheelSpeeds);
        try
        {
          wheelSpeeds->lfSpeed = std::stod(data[0]);
          wheelSpeeds->rfSpeed = std::stod(data[1]);
          wheelSpeeds->lbSpeed = std::stod(data[2]);
          wheelSpeeds->rbSpeed = std::stod(data[3]);
          wheelSpeeds->header.stamp = ros::Time::now();
          wheelSpeedsPub_.publish(wheelSpeeds);
        } catch (std::exception& e )
        {
          serialPort_.diag_warn("Converting wheel speeds data failed");
        }
      } else
      {
        serialPort_.diag_warn("Processing wheel speeds data failed");
      }
      break;
    }
    
    //RC input data
    case 'r':
    {
      std::vector<std::string> data;
      boost::split(data, msg, boost::is_any_of(","));
      if(data.size() == 4)
      {
        autorally_msgs::chassisCommandPtr chassisCommand(new autorally_msgs::chassisCommand);
        try
        {
          chassisCommand->steering = actuatorMsToCmd(std::stoi(data[0]), "steering");
          chassisCommand->throttle = actuatorMsToCmd(std::stoi(data[1]), "throttle");
          chassisCommand->frontBrake = -5.0;
          chassisCommand->sender = "RC";
          chassisCommand->header.stamp = ros::Time::now();
          chassisCommandPub_.publish(chassisCommand);

          chassisStateMutex_.lock();
          if(std::stoi(data[2]) > 1500)
          {
            autonomousEnabled_ = true;
          } else
          {
            autonomousEnabled_ = false;
          }
          throttleRelayEnabled_ = std::stoi(data[3]); //this value can be 0 or 1
          chassisStateMutex_.unlock();

        } catch(std::exception& e)
        {
          serialPort_.diag_warn("Converting wheel speeds data failed");
        }

      } else
      {
        serialPort_.diag_warn("Processing chassic RC data failed");
      }
      
      break;
    }
    //Castle Link ESC data, stores as 2-byte shorts in message
    case 'c':
    {
      short tmp;
      double val;
      double divisor = 2042;
      
      if (msg.length() < 21)
      {
        serialPort_.diag_warn("Receved too few bytes in ESC data " + std::to_string(msg.length()));
      }

      for(size_t i = 0; i < (msg.length()-2)/2; i++)
      {
        tmp = (msg[2*i]<<8) + msg[2*i+1];
        val = (tmp/divisor)*escRegisterData_[i].second;
        serialPort_.diag(escRegisterData_[i].first, std::to_string(val));
      }

      break;
    }    
    //error message
    case 'e':
    {
      serialPort_.diag_error(msg);
      break;
    }

    default:
    {
      serialPort_.diag_error("Unknown message type received from chassis:" + msgType);
      break;
    }
  }
}

void autorally_chassis::setChassisActuators(const ros::TimerEvent&)
{
  autorally_msgs::chassisStatePtr chassisState(new autorally_msgs::chassisState);
  
  chassisState->steeringCommander = "";
  chassisState->steering = 0.0;

  chassisState->throttleCommander = "";
  chassisState->throttle = 0.0;

  chassisState->frontBrakeCommander = "";
  chassisState->frontBrake = 0.0;

  ros::Time currentTime = ros::Time::now();

  //check if motion is enabled via from all valid runstop messages
  chassisState->runstopMotionEnabled = true;
  for(auto& runstop : runstops_)
  {
    if( currentTime-runstop.second.header.stamp < runstopMaxAge_)
    {
      if(runstop.second.motionEnabled == 0)
      {
        chassisState->runstopMotionEnabled = false;
        chassisState->throttleCommander = "runstop";
      }
    }
  }

  // find highest priority (lowest value) command message for each actuator across all valid actuator commands
  for(auto & vecIt : chassisCommandPriorities_)
  {
    if( currentTime-chassisCommands_[vecIt.id].header.stamp < chassisCommandMaxAge_)
    {
      //valid throttle commands are on [-1,1], only set throttle value if runstop is enabled
      if(chassisState->throttleCommander.empty() && chassisState->runstopMotionEnabled &&
         chassisCommands_[vecIt.id].throttle <= 1.0 &&
         chassisCommands_[vecIt.id].throttle >= -1.0)
      {
        chassisState->throttleCommander = chassisCommands_[vecIt.id].sender;
        chassisState->throttle = chassisCommands_[vecIt.id].throttle;
      }

      //valid steeringBrake commands are on [-1,1]
      if(chassisState->steeringCommander.empty() &&
         chassisCommands_[vecIt.id].steering <= 1.0 &&
         chassisCommands_[vecIt.id].steering >= -1.0)
      {
        chassisState->steeringCommander = chassisCommands_[vecIt.id].sender;
        chassisState->steering = chassisCommands_[vecIt.id].steering;
      }

      //valid frontBrake commands are on [0,1]
      if(chassisState->frontBrakeCommander.empty() &&
         chassisCommands_[vecIt.id].frontBrake <= 1.0 &&
         chassisCommands_[vecIt.id].frontBrake >= 0.0)
      {
        chassisState->frontBrakeCommander = chassisCommands_[vecIt.id].sender;
        chassisState->frontBrake = chassisCommands_[vecIt.id].frontBrake;
      }

    }
  }

  //send diagnostic info about who is in control of each actuator
  serialPort_.diag("throttle commander", chassisState->throttleCommander);
  serialPort_.diag("steering commander", chassisState->steeringCommander);
  serialPort_.diag("frontBrake commander", chassisState->frontBrakeCommander);

  //send actuator commands down to chassis, sets to calibrated neutral if no valid commander
  sendCommandToChassis(chassisState);

  chassisStateMutex_.lock();
  chassisState->throttleRelayEnabled = throttleRelayEnabled_;
  chassisState->autonomousEnabled = autonomousEnabled_;
  chassisStateMutex_.unlock();

  //publish state message
  chassisState->header.stamp = ros::Time::now();
  chassisState->header.frame_id = "autorally_chassis";
  chassisStatePub_.publish(chassisState);
  serialPort_.tick("chassisState");
}

void autorally_chassis::sendCommandToChassis(autorally_msgs::chassisStatePtr& state)
{
  //assemble send command message for chassis
  char actuatorCmd[8];
  short val;
  
  actuatorCmd[0] = 'c';
  //steering
  val = actuatorCmdToMs(state->steering, "steering");
  actuatorCmd[1] = val;

  //throttle
  val = actuatorCmdToMs(state->throttle, "throttle");
  actuatorCmd[3] = val;

  //frontBrake
  actuatorCmd[5] = val;
  val = actuatorCmdToMs(state->frontBrake, "frontBrake");

  actuatorCmd[7] = '\n';

  serialPort_.writePort(actuatorCmd);
}

short autorally_chassis::actuatorCmdToMs(double actuatorValue, std::string actuator)
{
  //convert actuator command message to raw PWM pulse width in ms using the actuator config
  short val = actuatorConfig_[actuator].center;
  if(actuatorValue < 0)
  {
    val += (short)((actuatorConfig_[actuator].center-actuatorConfig_[actuator].min)*actuatorValue);
  } else if(actuatorValue > 0)
  {
    val += (short)((actuatorConfig_[actuator].max-actuatorConfig_[actuator].center)*actuatorValue);
  }
  return val;
}

double autorally_chassis::actuatorMsToCmd(int pulseWidth, std::string actuator)
{
  //convert PWM pulse width in ms back to actuator command message using the actuator config
  int val = pulseWidth-actuatorConfig_[actuator].center;
  if(val < 0)
  {
    return val/(actuatorConfig_[actuator].center-actuatorConfig_[actuator].min);
  } else if(val > 0)
  {
    return val/(actuatorConfig_[actuator].max-actuatorConfig_[actuator].center);
  }
  return val;
}

void autorally_chassis::loadChassisConfig()
{
  //read in actuator settings
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  XmlRpc::XmlRpcValue v;
  nhPvt.param("actuators", v, v);
  if(v.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    XmlRpc::XmlRpcValue actuatorInfo;
    ActuatorConfig toAdd;
    for(int i = 0; i < v.size(); i++)
    {
      actuatorInfo = v[std::to_string(i)];

      if(actuatorInfo.getType() == XmlRpc::XmlRpcValue::TypeStruct &&
         actuatorInfo.size() == 5)
      {
        toAdd.center = static_cast<int>(actuatorInfo["center"]);
        toAdd.min = static_cast<int>(actuatorInfo["min"]);
        toAdd.max = static_cast<int>(actuatorInfo["max"]);
        toAdd.range = toAdd.max-toAdd.min;
        toAdd.port = i;
        toAdd.reverse = static_cast<bool>(actuatorInfo["reverse"]);
        actuatorConfig_[actuatorInfo["name"]] = toAdd;
      } else
      {
        NODELET_ERROR("autorally_chassis: XmlRpc actuator settings formatted incorrectly");
      }
    }
  } else
  {
    NODELET_ERROR("autorally_chassis: Couldn't retreive actuator settings");
  }
  NODELET_INFO("autorally_chassis: Loaded %lu actuators", actuatorConfig_.size());

}

void autorally_chassis::loadChassisCommandPriorities()
{
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  XmlRpc::XmlRpcValue v;
  nhPvt.param("chassisCommandProirities", v, v);
  std::map<std::string, XmlRpc::XmlRpcValue>::iterator mapIt;
  for(mapIt = v.begin(); mapIt != v.end(); mapIt++)
  {
    if(mapIt->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      //add entry in priority queue and command map
      priorityEntry toAdd;
      toAdd.id = mapIt->first;
      toAdd.priority = static_cast<int>(mapIt->second);
      chassisCommandPriorities_.push_back(toAdd);
      chassisCommands_[mapIt->first] = autorally_msgs::chassisCommand();

    } else
    {
      NODELET_ERROR("autorally_chassis: XmlRpc chassis command priorities formatted incorrectly");
    }
  }
  std::sort(chassisCommandPriorities_.begin(),
            chassisCommandPriorities_.end(),
            priorityComparator());

  std::vector<priorityEntry>::const_iterator vecIt;
  for(vecIt = chassisCommandPriorities_.begin();
      vecIt != chassisCommandPriorities_.end();
      vecIt++)
  {
    NODELET_INFO_STREAM("autorally_chassis chassisCommand ID:Priorities:" << vecIt->id << ":" << vecIt->priority);
  }
  NODELET_INFO_STREAM("autorally_chassis: Loaded " <<
                      chassisCommandPriorities_.size() << " chassis commanders");
}

}
