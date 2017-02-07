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
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDseparatedseparatedING, BUT NOT LIMITED TO, THE
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
 * @file AutoRallyChassis.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date July 10, 2016separated
 * @copyright 2016 Georgia Institute of Technology
 * @brief Interface for an AutoRally chassis
 *
 * @details Implementation of the AutoRallyChassis class that controls and receives state information from an AutoRally
 *          chassis
 ***********************************************/

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

#include <pluginlib/class_list_macros.h>

#include "AutoRallyChassis.h"

PLUGINLIB_DECLARE_CLASS(autorally_core, AutoRallyChassis, autorally_core::AutoRallyChassis, nodelet::Nodelet)

namespace autorally_core
{

AutoRallyChassis::~AutoRallyChassis()
{
  chassisControlTimer_.stop();
}

void AutoRallyChassis::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  std::string port;
  if(!nh.getParam(getName()+"/port", port))
  {
    NODELET_ERROR("AutoRallyChassis: could not get chassis interface port");
  }

  loadChassisConfig();
  loadChassisCommandPriorities();
  serialPort_.init(nh, getName(), "", "AutoRallyChassis", port, true);
  
  double commandRate = 0.0;
  double chassisCommandMaxAge = 0.0;
  double runstopMaxAge = 0.0;
  escDataFailCounter_ = 0;

  //need entry for each escDataFailCounter_actuator read from RC receiver to keep track of pulse statistics
  invalidActuatorPulses_["throttle"] = std::pair<bool, int>(false, 0);
  invalidActuatorPulses_["steering"] = std::pair<bool, int>(false, 0);

  if(!nhPvt.getParam("commandRate", commandRate) ||
     !nhPvt.getParam("commandMaxAge", chassisCommandMaxAge) ||
     !nhPvt.getParam("wheelDiameter", wheelDiameter_) ||
     !nhPvt.getParam("runstopMaxAge", runstopMaxAge))
  {
    NODELET_ERROR_STREAM(getName() << " could not get all startup params");
  }
  chassisCommandMaxAge_ = ros::Duration(chassisCommandMaxAge);
  runstopMaxAge_ = ros::Duration(runstopMaxAge);

  chassisStatePub_ = nh.advertise<autorally_msgs::chassisState>
                     ("chassisState", 1);
  wheelSpeedsPub_ = nh.advertise<autorally_msgs::wheelSpeeds>
                     ("wheelSpeeds", 1);
  chassisCommandPub_ = nh.advertise<autorally_msgs::chassisCommand>
                     ("RC/chassisCommand", 1);

  for (auto& mapIt : chassisCommands_)
  {
    std::string topic = mapIt.first+"/chassisCommand";
    ros::Subscriber sub = nh.subscribe(topic, 1,
                                       &AutoRallyChassis::chassisCommandCallback, this);
    chassisCommandSub_[mapIt.first] = sub;
  }
  runstopSub_ = nh.subscribe("/runstop", 5, &AutoRallyChassis::runstopCallback, this);  
  
  //callback for serial data from chassis
  serialPort_.registerDataCallback(boost::bind(&AutoRallyChassis::chassisFeedbackCallback, this));

  chassisControlTimer_ = nh.createTimer(ros::Rate(commandRate),
                      &AutoRallyChassis::setChassisActuators, this);
}

//subscribe to a one topic for every chassis commander listed in the chassis commander priorities file
void AutoRallyChassis::chassisCommandCallback(
                     const autorally_msgs::chassisCommandConstPtr& msg)
{
  std::map<std::string, autorally_msgs::chassisCommand>::iterator mapIt;
  if((mapIt = chassisCommands_.find(msg->sender)) == chassisCommands_.end())
  {
    NODELET_ERROR_STREAM("AutoRallyChassis: Unknown controller " <<
                         msg->sender <<
                         " attempting to control chassis, please add entry " <<
                         " to chassisCommandPriorities.yaml");
  }
  {
    mapIt->second = *msg;
  }
}

void AutoRallyChassis::chassisFeedbackCallback()
{
  //Any variables accessed in here and in other places in the code need to be mutex'd as this fires in a different
  //thread than the main thread. This is also a pretyt long callback, and ROS can shutdown underneath us, so check
  //if ROS system is still running any time anytseparatedhing ROS is used

  std::string data = "";
  size_t startPosition = 0;
  size_t endPosition = 0;

  //parse all messages from chassis
  do
  {
    serialPort_.lock();
    //look for start and end of message
    startPosition = serialPort_.m_data.find_first_of('#');
    endPosition = serialPort_.m_data.find_first_of('\n', startPosition);

    //pull message out of buffer if start and end are found
    if(startPosition != std::string::npos && endPosition != std::string::npos)
    {
      //frame data if not framed
      if(startPosition != 0)
      {
        serialPort_.m_data.erase(0, startPosition);
      }
      startPosition = 0;
      endPosition = serialPort_.m_data.find_first_of('\n', startPosition);

      //cut out and erase mescDataFailCounter_essage from queue
      data = serialPort_.m_data.substr(0, endPosition);
      serialPort_.m_data.erase(0, endPosition);
    }
    serialPort_.unlock();

    //process a complete messsage from the chassis with start ('#'), message type, and end ('\n') characters removed
    if(!data.empty())
    {
      processChassisMessage(data.substr(1,1), data.substr(2, endPosition-2));
      data.erase();
    }
  } while(endPosition != std::string::npos); //look for another message if we haven't looked at all data available yet
}

void AutoRallyChassis::processChassisMessage(std::string msgType, std::string msg)
{
  //ROS_INFO_STREAM(msgType << ":" << msg);
  switch(msgType[0])
  {
    //wheel speeds data as comma separated doubles, units in m/s
    case 'w':
    {
      std::vector<std::string> data;
      boost::split(data, msg, boost::is_any_of(","));
      if(data.size() == 4)
      {
        autorally_msgs::wheelSpeedsPtr wheelSpeeds(new autorally_msgs::wheelSpeeds);
        try
        {
          // Convert from rotations per second to m/s
          wheelSpeeds->lfSpeed = std::stod(data[0])*wheelDiameter_*PI;
          wheelSpeeds->rfSpeed = std::stod(data[1])*wheelDiameter_*PI;
          wheelSpeeds->lbSpeed = std::stod(data[2])*wheelDiameter_*PI;
          wheelSpeeds->rbSpeed = std::stod(data[3])*wheelDiameter_*PI;

          if(wheelSpeedsPub_ && !ros::isShuttingDown())
          {
            wheelSpeeds->header.stamp = ros::Time::now();
            wheelSpeedsPub_.publish(wheelSpeeds);
          }
          serialPort_.tick("wheelSpeeds data");
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
    
    //Actuator controls from RC input, as comma separated us pulse width, currentl frontBrake is not controlled by RC
    case 'r':
    {
      std::vector<std::string> data;
      boost::split(data, msg, boost::is_any_of(","));
      if(data.size() == 4)
      {
        autorally_msgs::chassisCommandPtr chassisCommand(new autorally_msgs::chassisCommand);
        try
        {
          //std::cout << std::stoi(data[0]) << " " << std::stoi(data[1]) << std::endl;
          chassisCommand->steering = actuatorUsToCmd(std::stoi(data[0]), "steering");
          chassisCommand->throttle = actuatorUsToCmd(std::stoi(data[1]), "throttle");
          chassisCommand->frontBrake = -5.0;
          chassisCommand->sender = "RC";
          //this line is in here for compatibility with the old servoInterface
          chassisCommand->header.frame_id = "RC";
          
          rcMutex_.lock();
          mostRecentRc_["frontBrake"] = chassisCommand->frontBrake;
          rcMutex_.unlock();
          
          if(chassisCommandPub_ && !ros::isShuttingDown())
          {
            chassisCommand->header.stamp = ros::Time::now();
            chassisCommandPub_.publish(chassisCommand);
          }

          chassisEnableMutex_.lock();
          if(std::stoi(data[2]) > 1500)
          {
            autonomousEnabled_ = true;
          } else
          {
            autonomousEnabled_ = false;
          }
          throttleRelayEnabled_ = std::stoi(data[3]); //this value can be 0 or 1
          chassisEnableMutex_.unlock();

          serialPort_.tick("RC data");
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
    //Castle Link ESC data, stored as 9, 2-byte shorts in message
    case 'c':
    {
      unsigned int tmp;
      double val;
      double divisor = 2042.0;
      
      if (msg.length() != 18)
      {
        escDataFailCounter_++;
        //Expected 18 bytes of ESC data, instead received " + std::to_string(msg.length()));
      }

      //std::cout << msg.length() << std::endl;
      for(size_t i = 0; i < msg.length()/2; i++)
      {
        tmp = ((((unsigned int)msg[2*i])&0xFF)<<8) + (((unsigned int)msg[2*i+1])&0xFF);
        
        //std::cout << escRegisterData_[i].second << " " << escRegisterData_[i].first << " " <<
        //             (((unsigned int)(msg[2*i]&0xFF))<<8) << " " << (int)(msg[2*i+1]&0xFF) << std::endl;
        
        val = (((double)tmp)/divisor)*escRegisterData_[i].second;
        
        serialPort_.diag(escRegisterData_[i].first, std::to_string(val));
      }
      serialPort_.diag("ESC data incorrect msg size counter", std::to_string(escDataFailCounter_));
      serialPort_.tick("ESC data");

      break;
    }    
    //error message as an ASCII string
    case 'e':
    {
      serialPort_.tick("Error message");
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

void AutoRallyChassis::setChassisActuators(const ros::TimerEvent&)
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
  if(runstops_.empty())
  {
    chassisState->runstopMotionEnabled = false;
  } else
  {
    chassisState->runstopMotionEnabled = true;
    int validRunstopCount = 0;
    for(auto& runstop : runstops_)
    {
      if(currentTime-runstop.second.header.stamp < runstopMaxAge_)
      {
        ++validRunstopCount;
        if(runstop.second.motionEnabled == 0)
        {
          chassisState->runstopMotionEnabled = false;
          chassisState->throttleCommander = "runstop";
        }
      }
    }
    if(validRunstopCount == 0)
    {
      chassisState->runstopMotionEnabled = false;
      chassisState->throttleCommander = "runstop";
      chassisState->throttle = 0.0;
    }
  }

  //find highest priority (lowest valuemostRecentRc_) command message for each actuator across all valid actuator commands
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

  //send actuator commands down to chassis, sets to calibrated neutral if no valid commander
  sendCommandToChassis(chassisState);

  chassisEnableMutex_.lock();
  chassisState->throttleRelayEnabled = throttleRelayEnabled_;
  chassisState->autonomousEnabled = autonomousEnabled_;
  chassisEnableMutex_.unlock();

  //send diagnostic info about who is in control of each actuator
  if(chassisState->autonomousEnabled)
  {
    //if we're in autonomous mode, set all the information apppropriately 
    serialPort_.diag("throttle commander", chassisState->throttleCommander);
    serialPort_.diag("steering commander", chassisState->steeringCommander);
    serialPort_.diag("frontBrake commander", chassisState->frontBrakeCommander);
  } else
  {
    //if we're in manual mode, send the most recentl RC command received from the chassis
    
    serialPort_.diag("throttle commander", "RC - manual");
    serialPort_.diag("steering commander", "RC - manual");
    serialPort_.diag("frontBrake commander", "RC - manual");
    rcMutex_.lock();
    chassisState->throttle = mostRecentRc_["throttle"];
    chassisState->throttleCommander = "RC - manual";
    chassisState->steering = mostRecentRc_["steering"];
    chassisState->steeringCommander = "RC - manual";
    chassisState->frontBrake = mostRecentRc_["frontBrake"];
    chassisState->frontBrakeCommander = "RC - manual";
    rcMutex_.unlock();
  }

  //publish state message
  if(chassisStatePub_ && !ros::isShuttingDown())
  {
    chassisState->header.stamp = ros::Time::now();
    chassisState->header.frame_id = "AutoRallyChassis";
    chassisStatePub_.publish(chassisState);
  }
  serialPort_.tick("chassisState pub");
}

void AutoRallyChassis::sendCommandToChassis(autorally_msgs::chassisStatePtr& state)
{
  /*
   * The message the chassis expects is 9 bytes:
   * byte 0: '#', the start delimiter
   * byte 1: 's', specifies message type
   * byte 2-3: steering pulse width in us packed into a short
   * byte 4-5: throttle pulse width in us packed into a short
   * byte 6-7: frontBrake pulse width in us packed into a short
   * byte 8: '\n', indicated end of message
   */

  //assemble send command message for chassis
  char actuatorCmd[9];
  short val;
  
  actuatorCmd[0] = '#';
  actuatorCmd[1] = 's';
  //steering
  val = actuatorCmdToMs(state->steering, "steering");
  actuatorCmd[2] = (char)((val&0xFF00)>>8);
  actuatorCmd[3] = (char)(val&0x00FF);

  //throttle
  val = actuatorCmdToMs(state->throttle, "throttle");
  actuatorCmd[4] = (char)((val&0xFF00)>>8);
  actuatorCmd[5] = (char)(val&0x00FF);

  //frontBrake
  val = actuatorCmdToMs(state->frontBrake, "frontBrake");
  actuatorCmd[6] = (char)((val&0xFF00)>>8);
  actuatorCmd[7] = (char)(val&0x00FF);

  //message end signal
  actuatorCmd[8] = '\n';

  serialPort_.writePort(actuatorCmd);
}

short AutoRallyChassis::actuatorCmdToMs(double actuatorValue, std::string actuator)
{
  //convert actuator command message to raw PWM pulse width in us using the actuator config

  //don't need to check if actuatorValue is on [-1, 1] because it was already done
  short val = actuatorConfig_[actuator].center;
  if(actuatorValue < 0)
  {
    val += (short)((actuatorConfig_[actuator].center-actuatorConfig_[actuator].min)*actuatorValue);
  } else if(actuatorValue >= 0)
  {
    val += (short)((actuatorConfig_[actuator].max-actuatorConfig_[actuator].center)*actuatorValue);
  }
  return val;
}

double AutoRallyChassis::actuatorUsToCmd(int pulseWidth, std::string actuator)
{
  double cmd = std::numeric_limits<double>::quiet_NaN();
  //convert PWM pulse width in us back to actuator command message using the actuator config
  
  //if us value is outside normal servo ranges, complain and don't try to convert
  if(pulseWidth < 900 || pulseWidth > 2100)
  {
    if(invalidActuatorPulses_[actuator].first == true)
    {
      serialPort_.diag_error(
        "Received multiple pulse widths out of valid range 900-2100ms in a row (" +
        std::to_string(pulseWidth) + ") from " + actuator );
            
      //we've gone 2 cycles without a valid reading, disable RC control of this actuator
      cmd = -5.0;
    } else
    {
      //if we only get one invalid pulse width in a row, just use the previous one
      cmd = mostRecentRc_[actuator];
      //only increment invalid pulses when we get one in a row, not continuously
      invalidActuatorPulses_[actuator].second++;
    }
    invalidActuatorPulses_[actuator].first = true;
  
    //NODELET_ERROR_STREAM(getName() << " RC " << actuator << " pulse width out of valid range 900-2100ms (" <<
    //                     pulseWidth << ")");
  } else
  {
    invalidActuatorPulses_[actuator].first = false;

    int val = pulseWidth-actuatorConfig_[actuator].center;
    if(val < 0)
    {
      cmd = val/((double)actuatorConfig_[actuator].center-actuatorConfig_[actuator].min);
    } else
    {
      cmd = val/((double)actuatorConfig_[actuator].max-actuatorConfig_[actuator].center);
    }

    //save most recent valid actuator command    
    mostRecentRc_[actuator] = cmd;
  }
  serialPort_.diag(actuator+ " single invalid pulse count",
                   std::to_string(invalidActuatorPulses_[actuator].second));
  return cmd;
}

void AutoRallyChassis::loadChassisConfig()
{
  //read in actuator settings from paramter server that were loaded from the launch file
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  XmlRpc::XmlRpcValue v;
  nhPvt.param("actuators", v, v);
  if(v.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    XmlRpc::XmlRpcValue actuatorInfo;
    ActuatorConfig toAdd;

    //add an entry for each actuator (steering, throttle, front brake)
    for(auto actuatorInfo : v)
    {
      if(actuatorInfo.second.getType() == XmlRpc::XmlRpcValue::TypeStruct &&
         actuatorInfo.second.size() == 4)
      {
        toAdd.center = static_cast<int>(actuatorInfo.second["center"]);
        toAdd.min = static_cast<int>(actuatorInfo.second["min"]);
        toAdd.max = static_cast<int>(actuatorInfo.second["max"]);
        toAdd.reverse = static_cast<bool>(actuatorInfo.second["reverse"]);
        actuatorConfig_[actuatorInfo.first] = toAdd;

        NODELET_INFO_STREAM(getName() << " loaded actuator " << actuatorInfo.first);
      } else
      {
        NODELET_ERROR_STREAM(getName() << " XmlRpc actuator settings formatted incorrectly");
      }
    }
  } else
  {
    NODELET_ERROR_STREAM(getName() << " Couldn't retreive actuator settings");
  }
  NODELET_INFO_STREAM(getName() << " loaded " << actuatorConfig_.size() << " actuators");

}

void AutoRallyChassis::loadChassisCommandPriorities()
{
  //read in chassisCommandPriorities from the parameter server that were loaded by the launch file
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
      NODELET_ERROR_STREAM(getName() << " XmlRpc chassis command priorities formatted incorrectly");
    }
  }

  //sort the loaded commanders according to their priority
  std::sort(chassisCommandPriorities_.begin(),
            chassisCommandPriorities_.end(),
            priorityComparator());

  std::vector<priorityEntry>::const_iterator vecIt;
  for(vecIt = chassisCommandPriorities_.begin();
      vecIt != chassisCommandPriorities_.end();
      vecIt++)
  {
    NODELET_INFO_STREAM(getName() << " loaded commander " << vecIt->id << " with priority " << vecIt->priority);
  }
  NODELET_INFO_STREAM(getName() << " loaded " <<
                      chassisCommandPriorities_.size() << " chassis commanders");
}

}
