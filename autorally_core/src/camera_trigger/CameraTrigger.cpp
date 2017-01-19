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
 * @file CameraTrigger.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 22, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Brief file description
 *        Brief description continued.
 *
 * @details Contains ArdionoOnboard class implementation
 ***********************************************/
#include "CameraTrigger.h"

#include <pluginlib/class_list_macros.h>

#include<boost/lexical_cast.hpp>

#include <numeric>

PLUGINLIB_DECLARE_CLASS(autorally_core, CameraTrigger, autorally_core::CameraTrigger, nodelet::Nodelet)

namespace autorally_core
{

CameraTrigger::CameraTrigger()
{}

CameraTrigger::~CameraTrigger()
{}

void CameraTrigger::onInit()
{
  ros::NodeHandle m_nhPvt = getPrivateNodeHandle();
 	std::string port;

  if(!m_nhPvt.getParam("port", port))
  {
      NODELET_ERROR("CameraTrigger: Could not get all CameraTrigger parameters");
  }

  //set up dynamic_reconfigure server
  dynamic_reconfigure::Server<camera_trigger_paramsConfig>::CallbackType cb;
  cb = boost::bind(&CameraTrigger::configCallback, this, _1, _2);
  m_dynReconfigServer.setCallback(cb);

	m_port.init(m_nhPvt, getName(), "", "CameraTrigger", port, true);
	m_port.registerDataCallback(boost::bind(&CameraTrigger::triggerDataCallback, this));
}

void CameraTrigger::triggerDataCallback()
{
  std::string msg;

  while(findMessage(msg))
  {
    boost::char_separator<char> seps(":,\n");
    tokenizer tok(msg, seps);
    tokenizer::iterator it=tok.begin();

    while(it!=tok.end())
    {
      if(*it == "pps")
      {
        if(++it == tok.end()) break;
        if(*it == "0")
        {
          m_port.diag_ok("PPS input good");
        }
        else if(*it == "1")
        {
          m_port.diag_warn("no PPS");
        }
        else
        {
          m_port.diag_error("unknown PPS status from trigger:" + *it);
        }
        m_port.tick("pps info");
      } else
      {
        NODELET_ERROR("CameraTrigger: Bad token %s", it->c_str());
        m_port.diag_warn("CameraTrigger got a bad token");
      }

      if(it!=tok.end())
      {
        ++it;
      }
    }
  }
  m_port.diag("Triggering FPS", std::to_string(m_triggerFPS));
}

bool CameraTrigger::findMessage(std::string& msg)
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

void CameraTrigger::configCallback(const camera_trigger_paramsConfig &config, uint32_t /*level*/)
{
  m_triggerFPS = config.camera_trigger_frequency;  
  //NODELET_ERROR_STREAM("Triggering FPS " << m_triggerFPS);  
  m_port.diag("Triggering FPS", std::to_string(m_triggerFPS));

  //send new FPS to arduino
  m_port.lock();
  m_port.writePort("#fps:" + std::to_string(m_triggerFPS) + "\r\n");
  m_port.unlock();  
}

}
