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
 * @file Diagnostics.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief Diagnostics class implementation
 *
 ***********************************************/

#include "autorally_core/Diagnostics.h"

#include <diagnostic_updater/publisher.h>
#include <ros/time.h>
#include <stdio.h>
#include <sstream>

Diagnostics::Diagnostics()
{}

Diagnostics::Diagnostics(const std::string otherInfo,
                         const std::string hardwareID,
                         const std::string hardwareLocation) :
  m_hardwareLocation(hardwareLocation),
  m_overallLevel(diagnostic_msgs::DiagnosticStatus::OK)
{
  init(otherInfo, hardwareID, hardwareLocation);
}

Diagnostics::~Diagnostics()
{}


void Diagnostics::init(const std::string& otherInfo,
                       const std::string& hardwareID,
                       const std::string& hardwareLocation)
{
  ros::NodeHandle nh;
  m_hardwareLocation = hardwareLocation;
  m_overallLevel = diagnostic_msgs::DiagnosticStatus::OK;
  m_updater.setHardwareID(hardwareID);
  m_updater.add(otherInfo, this, &Diagnostics::diagnostics);

  //can retrieve a global diagnosticsFrequency parameter if diagnostics should
  //be published at a differenc frequency than 1.0 second
  double diagFreq;
  ros::param::param<double>("diagnosticsFrequency", diagFreq, 1.0);
  m_heartbeatTimer = nh.createTimer(ros::Duration(diagFreq),
                      &Diagnostics::diagUpdate, this);

  m_statusTimer = nh.createTimer(ros::Duration(diagFreq),
                      &Diagnostics::diagnosticStatus, this);

}

void Diagnostics::diag(const std::string key, const std::string value, bool lock)
{
  if (lock) m_dataMutex.lock();
  m_diags[key] = value;
  if (lock) m_dataMutex.unlock();
}

void Diagnostics::diag_ok(const std::string msg)
{
  m_dataMutex.lock();
  m_diagMsgs[msg] = diagnostic_msgs::DiagnosticStatus::OK;
  m_dataMutex.unlock();
}

void Diagnostics::diag_warn(const std::string msg)
{
  m_dataMutex.lock();
  m_diagMsgs[msg] = diagnostic_msgs::DiagnosticStatus::WARN;
  m_dataMutex.unlock();
}

void Diagnostics::diag_error(const std::string msg)
{
  m_dataMutex.lock();
  m_diagMsgs[msg] = diagnostic_msgs::DiagnosticStatus::ERROR;
  m_dataMutex.unlock();
}

void Diagnostics::diagUpdate(const ros::TimerEvent&)
{
  //force the publishing of a diagnostics array based on the desired frequency
  m_updater.force_update();
}

void Diagnostics::OK()
{
  m_dataMutex.lock();
  m_overallLevel = diagnostic_msgs::DiagnosticStatus::OK;
  m_dataMutex.unlock();
}

void Diagnostics::WARN()
{
    m_dataMutex.lock();
  m_overallLevel = diagnostic_msgs::DiagnosticStatus::WARN;
  m_dataMutex.unlock();
}

void Diagnostics::ERROR()
{
  m_dataMutex.lock();
  m_overallLevel = diagnostic_msgs::DiagnosticStatus::ERROR;
  m_dataMutex.unlock();
}

void Diagnostics::tick(const std::string &name)
{
  std::map<std::string, std::vector<std::pair<int, ros::Time> > >::iterator mapIt;
  m_dataMutex.lock();
  if( (mapIt = m_ticks.find(name)) == m_ticks.end())
  {
    std::vector<std::pair<int, ros::Time> > toAdd;
    toAdd.push_back(std::pair<int, ros::Time>(0, ros::Time::now()) );

    mapIt = m_ticks.insert(std::pair<std::string,
                           std::vector<std::pair<int, ros::Time> > >
                           (name, toAdd)).first;
  }
  ++mapIt->second.back().first;
  m_dataMutex.unlock();
}

void Diagnostics::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  //add current overall diagnostic level and message
  stat.summary(m_overallLevel, m_hardwareLocation);

  //Frequency messages are added to diagnotics only if tick() is being called
  std::map<std::string, std::vector<std::pair<int, ros::Time> > >::iterator mapItF;
  ros::Time n = ros::Time::now();
  m_dataMutex.lock();
  for( mapItF = m_ticks.begin(); mapItF != m_ticks.end(); ++mapItF)
  {
    //remove all tick counts older than 15 seconds
    while( (n-mapItF->second.front().second).toSec() > 20.0)
    {
      mapItF->second.erase(mapItF->second.begin());
    }

    //sum all ticks in the window
    std::vector<std::pair<int, ros::Time> >::const_iterator qIt;
    int sum = 0;
    for( qIt = mapItF->second.begin(); qIt != mapItF->second.end(); ++qIt)
    {
      sum += qIt->first;
    }
    
    //add a diagnostic message with the publishing freq over the sliding window
    double val = sum/(n-mapItF->second.front().second).toSec();
    if(!std::isnan(val) && !std::isinf(val))
    { //strs << sum/((n-mapItF->second.front().second).toSec());
      diag(mapItF->first + " freq(hz):", std::to_string(val), false);
    }
    //add new tick entry to vector
    mapItF->second.push_back( std::pair<int, ros::Time>(0, n));
  }

  //add all queued diganostic messages, clear the queues
  std::map<std::string, char>::iterator mapIt;
  for(mapIt = m_diagMsgs.begin(); mapIt != m_diagMsgs.end(); ++mapIt)
  {
    stat.add(mapIt->first, mapIt->second);
  }
  m_diagMsgs.clear();

  std::map<std::string, std::string>::iterator mapIt2;
  for(mapIt2 = m_diags.begin(); mapIt2 != m_diags.end(); ++mapIt2)
  {
    stat.add(mapIt2->first, mapIt2->second);
  }
  m_diags.clear();
  m_dataMutex.unlock();
}
