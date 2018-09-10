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
 * @file ArduinoOnboard.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 22, 2014
 * @copyright 2014 Georgia Institute of Technology
 * @brief Brief file description
 *        Brief description continued.
 *
 * @details Contains ArdionoOnboard class implementation
 ***********************************************/
#include "ArduinoOnboard.h"

#include <pluginlib/class_list_macros.h>

#include<boost/lexical_cast.hpp>

#include <numeric>

PLUGINLIB_EXPORT_CLASS( autorally_core::ArduinoOnboard,  nodelet::Nodelet)

namespace autorally_core
{

ArduinoOnboard::ArduinoOnboard():
  m_lfSum(0.0),
  m_rfSum(0.0),
  m_lbSum(0.0),
  m_rbSum(0.0)
{}

ArduinoOnboard::~ArduinoOnboard()
{

}

void ArduinoOnboard::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  m_nhPvt = getPrivateNodeHandle();

  m_lastTime = ros::Time::now();
 	std::string port;

  if(!m_nhPvt.getParam("port", port) ||
     !m_nhPvt.getParam("numMovingAverageValues", m_numMovingAverageValues) ||
     !m_nhPvt.getParam("wheelDiameter", m_wheelDiameter) ||
     !m_nhPvt.getParam("triggerFPS", m_triggerFPS) ||
     !m_nhPvt.getParam("srvBatteryCrit", srvBatteryCrit) ||
     !m_nhPvt.getParam("srvBatteryLow", srvBatteryLow) ||
     !m_nhPvt.getParam("camBatteryCrit", camBatteryCrit) ||
     !m_nhPvt.getParam("camBatteryLow", camBatteryLow) ||
     !m_nhPvt.getParam("lfRotationEnabled", m_lfEnabled) ||
     !m_nhPvt.getParam("rfRotationEnabled", m_rfEnabled) ||
     !m_nhPvt.getParam("lbRotationEnabled", m_lbEnabled) ||
     !m_nhPvt.getParam("rbRotationEnabled", m_rbEnabled) )
  {
      ROS_ERROR("ArduinoOnboard: Could not get all arduinoOnboard parameters");
  }

  m_wheelSpeedPub = nh.advertise<autorally_msgs::wheelSpeeds>
                                    ("wheelSpeeds", 1);

  m_servoPub = nh.advertise<autorally_msgs::chassisCommand> ("RC", 1);

	m_port.init(m_nhPvt, getName(), "", "ArduinoOnboard", port, true);
	m_port.registerDataCallback(
                      boost::bind(&ArduinoOnboard::arduinoDataCallback, this));

  loadServoParams();
}

void ArduinoOnboard::arduinoDataCallback()
{
  std::string msg;

// These are super expensive on the jetson
//  m_nhPvt.getParam("srvBatteryCrit", srvBatteryCrit);
//  m_nhPvt.getParam("srvBatteryLow", srvBatteryLow);
//  m_nhPvt.getParam("camBatteryCrit", camBatteryCrit);
//  m_nhPvt.getParam("camBatteryLow", camBatteryLow);

  while(findMessage(msg))
  {
    //allocate new wheelSpeeds message
    autorally_msgs::wheelSpeedsPtr wheelSpeeds(new autorally_msgs::wheelSpeeds);
    wheelSpeeds->header.stamp = ros::Time::now();

    boost::char_separator<char> seps(":,\n");
    tokenizer tok(msg, seps);
    tokenizer::iterator it=tok.begin();

    //allocate new servo message for RC servoCommand
    autorally_msgs::chassisCommandPtr servos(new autorally_msgs::chassisCommand);
    servos->sender = "RC";
    servos->frontBrake = -5.0;

    double lf, rf, lb, rb;
    bool wheels = false;
    //bool servo = false;
    //bool camera = false;
    bool rc = false;

    while(it!=tok.end())
    {
      if(*it == "wheels")
      {
        if(++it == tok.end()) break;
        lf = atof(it->c_str());

        if(++it == tok.end()) break;
        rf = atof(it->c_str());

        if(++it == tok.end()) break;
        lb = atof(it->c_str());

        if(++it == tok.end()) break;
        rb = atof(it->c_str());

        wheels = true;
      //} else if(*it == "servo")
      //{
      //  if(++it == tok.end()) break;
      //  m_port.diag("Servo Battery Voltage", *it);
      //  servo = true;
      //} else if(*it == "camera")
      //{
      //  if(++it == tok.end()) break;
      //  m_port.diag("Camera Battery Voltage", *it);
      //  camera = true;
      } else if(*it == "rc")
      {
        if(++it == tok.end()) break;
        servos->throttle = atof(it->c_str());

        if(++it == tok.end()) break;
        servos->steering = atof(it->c_str());
        rc = true;
      } else
      {
        NODELET_ERROR("ArduinoOnboard: Bad token %s", it->c_str());
        m_port.diag_warn("ArduinoOnboard got a bad token");
      }

      if(it!=tok.end())
      {
        ++it;
      }
    }
    if(!(wheels && rc))
    {
      NODELET_ERROR("ArduinoOnboard: Incomplete packet %d %d", wheels, rc);
      m_port.diag_warn("ArduinoOnboard: Incomplete packet");
    }

    if(std::isnan(lf)) lf = 0.0;
    if(std::isnan(lb)) lb = 0.0;
    if(std::isnan(rf)) rf = 0.0;
    if(std::isnan(rb)) rb = 0.0;

    wheelSpeeds->lfSpeed = (lf)*m_wheelDiameter*PI;
    wheelSpeeds->lbSpeed = (lb)*m_wheelDiameter*PI;
    wheelSpeeds->rfSpeed = (rf)*m_wheelDiameter*PI;
    wheelSpeeds->rbSpeed = (rb)*m_wheelDiameter*PI;

    //check if one of the front rotation sensors is disabled
    if(m_lfEnabled && !m_rfEnabled)
    {
      wheelSpeeds->rfSpeed = wheelSpeeds->lfSpeed;
    } else if(!m_lfEnabled && m_rfEnabled)
    {
      wheelSpeeds->lfSpeed = wheelSpeeds->rfSpeed;
    }

    //check if one of the back rotation sensors is disabled
    if(m_lbEnabled && !m_rbEnabled)
    {
      wheelSpeeds->rbSpeed = wheelSpeeds->lbSpeed;
    } else if(!m_lbEnabled && m_rbEnabled)
    {
      wheelSpeeds->lbSpeed = wheelSpeeds->rbSpeed;
    }

    //Calculate servo commands
    //Raw data from arduino is in units of 500 ns.
    servos->throttle /= 2.0;
    servos->steering /= 2.0;
    std::map<std::string, ServoSettings>::const_iterator mapIt;

    if( (mapIt = m_servoSettings.find("steering")) != m_servoSettings.end())
    {
      if(servos->steering > mapIt->second.center)
      {
        servos->steering = (servos->steering - mapIt->second.center) / (mapIt->second.max-mapIt->second.center);
        if (servos->steering > 1.0)
            servos->steering = 1.0;
      } else if(servos->steering < mapIt->second.center)
      {
        servos->steering = (servos->steering-mapIt->second.center)/(mapIt->second.center-mapIt->second.min);
        if (servos->steering < -1.0)
            servos->steering = -1.0;
      } else
      {
        servos->steering = 0.0;
      }

      if(mapIt->second.reverse)
      {
        servos->steering = -servos->steering;
      }
    }
    if( (mapIt = m_servoSettings.find("throttle")) != m_servoSettings.end())
    {
      if(servos->throttle > mapIt->second.center)
      {
        servos->throttle = (servos->throttle - mapIt->second.center) / (mapIt->second.max-mapIt->second.center);
        if (servos->throttle > 1.0)
            servos->throttle = 1.0;
      } else if(servos->throttle < mapIt->second.center)
      {
        servos->throttle = (servos->throttle-mapIt->second.center)/(mapIt->second.center-mapIt->second.min);
        if (servos->throttle < -1.0)
            servos->throttle = -1.0;
      } else
      {
        servos->throttle = 0.0;
      }

      if(mapIt->second.reverse)
      {
        servos->throttle = -servos->throttle;
      }
    }

    
    servos->header.stamp = ros::Time::now();

    if (m_lastTime + ros::Duration(2.0) > ros::Time::now()) {
      m_lastTime = ros::Time::now();
      //set FPS
      int newFPS = 0;
      m_nhPvt.getParam("triggerFPS", newFPS);
      if(newFPS != m_triggerFPS)
      {
        m_triggerFPS = newFPS;
        m_port.lock();
        m_port.writePort("#fps:" + std::to_string(m_triggerFPS) + "\r\n");
        m_port.unlock();
      }
    }

    //publish data
    m_wheelSpeedPub.publish(wheelSpeeds);
    m_servoPub.publish(servos);
    m_port.tick("arduinoData");
    m_port.diag("Triggering FPS", std::to_string(m_triggerFPS));
  }
}

bool ArduinoOnboard::findMessage(std::string& msg)
{
  m_port.lock();
  if(m_port.m_data.size() > 10)
  {
    //make sure data is framed
    if(m_port.m_data[0] != '#')
    {
      size_t start = m_port.m_data.find("#");
      m_port.m_data.erase(0, start);
    }

    //try to pull out a full message
    size_t end = m_port.m_data.find("\r\n");
    if(end != std::string::npos)
    {
      //std::cout << "Looking at^" << m_port.m_data << "^" << std::endl;
      //remove # at beginning and trailing \r\n before further processing
      msg = m_port.m_data.substr(1,end-1);
      //erase through \r\n at end of message
      m_port.m_data.erase(0,end+2);
      m_port.unlock();
      return true;
    }
  }
  m_port.unlock();
  return false;
}

void ArduinoOnboard::loadServoParams()
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
  NODELET_INFO("ServoInterface: Loaded %u servos", (unsigned int)m_servoSettings.size());

}


}
