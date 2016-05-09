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
 * @file SafeSpeed.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief
 *
 ***********************************************/

#include <math.h>

#include <autorally_core/SafeSpeed.h>

namespace autorally_core
{

SafeSpeed::SafeSpeed():
  m_vehicleSpeeds(15), //keep 15 most recent speed values
  m_prevGoodThrottle(0.0),
  m_safeSpeedIsInControl(false)
{}

SafeSpeed::~SafeSpeed()
{}

void SafeSpeed::onInit()
{
}

void SafeSpeed::init(ros::NodeHandle &nh)
{
  m_safeSpeedSub = nh.subscribe("safeSpeed", 5,
                                &SafeSpeed::safeSpeedCallback,
                                this);
  m_speedSub = nh.subscribe("wheelSpeeds", 5,
                            &SafeSpeed::wheelSpeedsCallback,
                            this);
  if(!nh.getParam("safeSpeed/maxSafeSpeed", m_maxSafeSpeed) ||
     !nh.getParam("safeSpeed/Timeout", m_safeSpeedTimeout) )
  {
    ROS_ERROR("SafeSpeed: could not get all parameters");
  }

  loadThrottleCalibration(nh);
}

void SafeSpeed::safeSpeedCallback(
  const autorally_msgs::safeSpeedConstPtr& msg)
{
  //update SafeSpeed from sender, and timstamp assosiated with it
  std::map<std::string, SafeSpeedData>::iterator newSafeSpeed;
  if( (newSafeSpeed = m_safeSpeeds.find(msg->sender)) != m_safeSpeeds.end())
  {
    newSafeSpeed->second.time = msg->header.stamp;
    newSafeSpeed->second.safeSpeed = msg->speed;
  } else
  { //if there is not yet an entry to sender, add it
    SafeSpeedData toAdd;
    toAdd.time = msg->header.stamp;
    toAdd.safeSpeed = msg->speed;

    m_safeSpeeds.insert(std::pair<std::string,
                        SafeSpeedData>(
                        msg->sender, toAdd));
  }
}

double SafeSpeed::getSafeSpeed() const
{
  double safeSpeed = m_maxSafeSpeed;
  std::map<std::string, SafeSpeedData>::const_iterator mapIt;
  ros::Time now = ros::Time::now();

  bool foundValidSafeSpeed = false;
  for(mapIt = m_safeSpeeds.begin(); mapIt != m_safeSpeeds.end(); mapIt++)
  {
    //if the SafeSpeed is still considered valid
    if( now-mapIt->second.time < ros::Duration(m_safeSpeedTimeout))
    {
      safeSpeed = std::min(safeSpeed, mapIt->second.safeSpeed);
      foundValidSafeSpeed = true;
    }
  }

  return (foundValidSafeSpeed) ? safeSpeed : -1.0;
}

void SafeSpeed::setMaxSpeed(const double &maxSafeSpeed)
{
  m_maxSafeSpeed = maxSafeSpeed;
}

double SafeSpeed::maxSpeed() const
{
  return m_maxSafeSpeed;
}

void SafeSpeed::wheelSpeedsCallback(const autorally_msgs::wheelSpeedsConstPtr& msg)
{

  WheelData toAdd;
  toAdd.msg = *msg;
  toAdd.speed = (msg->lfSpeed + msg->rfSpeed)/2.0;
  /* Only use front wheels in case back wheels are spinning out.
   * Since this is a fixed-size circular buffer, old measurements will be
   * pushed out as new data arrives.
   */
  m_vehicleSpeeds.push_back(toAdd);
}

double SafeSpeed::safeThrottle(const double& throttleCommand)
{
  double safeSpeed = getSafeSpeed();
  if(safeSpeed <= 0.0 || m_vehicleSpeeds.size() < 2)
  {
    return 0.0;
  }

  //check if safeSpeed can give up control (if it's in control)
  if(m_safeSpeedIsInControl && throttleCommand < m_prevGoodThrottle)
  {
    m_safeSpeedIsInControl = false;
  }

  double safeThrottle = 0.0;
  if(!m_safeSpeedIsInControl)
  {
    //calculate acceleration
/*    double acceleration = 0.0;
    double timeDiff;
    boost::circular_buffer<WheelData>::const_iterator vecItC = m_vehicleSpeeds.begin();
    boost::circular_buffer<WheelData>::const_iterator vecItP = vecItC++;
    while(vecItC != m_vehicleSpeeds.end())
    {
      if(vecItC->msg.header.stamp != vecItP->msg.header.stamp)
      {
        timeDiff = (vecItC->msg.header.stamp-vecItP->msg.header.stamp).toSec();
        acceleration += (vecItC->speed-vecItP->speed)/timeDiff;
      }
      vecItP = vecItC++;
    }
    NODELET_WARN_STREAM(" Acc:" << acceleration << " Size:" << m_vehicleSpeeds.size());
    acceleration /= m_vehicleSpeeds.size();
*/
    //if all is good, let the throttle do whatever
    //NODELET_WARN_STREAM("Speed:" << m_vehicleSpeeds.back().speed << " Acc:" << acceleration << " SafeSpeed:" << safeSpeed); 
    if(m_vehicleSpeeds.back().speed/*+acceleration*/ < safeSpeed)
    {
      safeThrottle = throttleCommand;
    } else
    {
      m_prevGoodThrottle = throttleCommand;
      m_safeSpeedIsInControl = true;
    }
  }

  if(m_safeSpeedIsInControl)
  {
    // cut throttle, act as a speed governer

    safeThrottle = 0.0;
    //compute safeThrottle
    //if(!m_throttleMappings.interpolateKey(safeSpeed, safeThrottle))
    //{
    //  safeThrottle = 0.0;
    //  NODELET_WARN_STREAM(
    //            "SafeSpeed: couldn't interpolate a safeThrottle from safeSpeed:"
    //            << safeSpeed);
    //}
  }

  return safeThrottle;
}

void SafeSpeed::loadThrottleCalibration(ros::NodeHandle &nh)
{
  //ros::NodeHandle nhPvt = getPrivateNodeHandle();
  XmlRpc::XmlRpcValue v;
  nh.param("ServoInterface/throttleCalibration", v, v);
  std::map<std::string, XmlRpc::XmlRpcValue>::iterator mapIt;
  for(mapIt = v.begin(); mapIt != v.end(); mapIt++)
  {
    if(mapIt->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      std::pair<double, double> toAdd(std::pair<double, double>(
                                      boost::lexical_cast<double>(mapIt->first),
                                      static_cast<double>(mapIt->second)));
      if(!m_throttleMappings.update(toAdd))
      {
        NODELET_ERROR_STREAM("SafeSpeed: Failed to add mapping " <<
                             toAdd.first << ":" << toAdd.second);
      }
    } else
    {
      NODELET_ERROR("SafeSpeed: XmlRpc throttle calibration formatted incorrectly");
    }
  }
  NODELET_INFO_STREAM("SafeSpeed: Loaded " << m_throttleMappings.size() <<
                      " throttle mappings");
}

}
