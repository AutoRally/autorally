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
 * @file ChronyStatus.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief ChronyStatus class implementation
 *
 ***********************************************/

#include <autorally_core/ChronyStatus.h>
#include <fstream>

ChronyStatus::ChronyStatus(ros::NodeHandle &nh,
                            const std::string nodeName,
                            const std::string hardwareID,
                            const std::string port) :
  Diagnostics(nodeName, hardwareID, port),
  m_masterHostname(getenv("MASTER_HOSTNAME")),
  m_hostname(getenv("HOSTNAME"))
{
  double updateClientListFrequency;
  ros::param::param<double>("updateClientListFrequency",
                            updateClientListFrequency,
                            5.0);

  m_updateClientsTimer = nh.createTimer(ros::Duration(updateClientListFrequency),
                      &ChronyStatus::updateClients, this);
}

ChronyStatus::~ChronyStatus()
{}

void ChronyStatus::updateClients(const ros::TimerEvent& /*time*/)
{
  int replyStatus;
  char line[150];
  char client[50];
  if(m_masterHostname != m_hostname)
  {
    if(system(NULL))
    {
      if(system("~/ros_workspace/src/auto_rally/autorally_core/src/ChronyStatus/getClients.sh"))
      {
        diag_warn("system() failed");
        return;
      }

      m_clients.clear();

      std::ifstream ifs("chronyClients");
      ifs.getline(line, 150);

      sscanf(line, "%d %s", &replyStatus, client);
      if(replyStatus != 200 || strcmp(client,"OK") != 0)
      {
        diag_error("Password not accepted");
      } else
      {
        ifs.getline(line, 150);
        ifs.getline(line, 150);

        //parse out all the clients
        while(!ifs.eof())
        {
          ifs.getline(line, 150);
          if(sscanf(line, "%s ", client) == 1)
          {
            //std::cout << client << "||" << line << std::endl;

            if(strcmp(client, "localhost") != 0)
            {
              m_clients.push_back(client);
            }
          }
        }
      }
    } else
    {
      diag_warn("system() unavailable");
    }
  } else
  {
    diag_warn("Time sync info unavailable");
  }
}

void ChronyStatus::diagnosticStatus(const ros::TimerEvent& /*time*/)
{
  char line[150];
  char client[50];
  char clientRequest[200];
  int sys = 0;
  int replyStatus;
  std::vector<std::string>::iterator vecIt;

  if(!m_clients.empty())
  {
    if(system(NULL))
    {
      sys += system("rm -rf chronySourceStats");

      for(vecIt = m_clients.begin(); vecIt != m_clients.end(); vecIt++)
      {
	      if(m_hostname == *vecIt)
	      {
          sprintf(clientRequest, "chronyc sourcestats > chronySourceStats");
        } else
        {
          sprintf(clientRequest,
                "chronyc -h %s sourcestats > chronySourceStats",
                vecIt->c_str());
        }
        sys += system(clientRequest);
      }

      if(sys > 0)
      {
        diag_warn("system() failed");
        return;
      }

      std::ifstream timing("chronySourceStats", std::ifstream::in);

      while(!timing.eof())
      {
        timing.getline(line, 200);
        sscanf(line, "%d ", &replyStatus);
        if(replyStatus != 210)
        {
          diag_error("Could not read sourcestats");
        } else
        {
          timing.getline(line, 200);
          timing.getline(line, 200);
          char frequency[30];
          char frequencySkew[30];
          char offset[30];
          char stddev[30];

          timing.getline(line, 200);
          //std::cout << line << std::endl;
          sscanf(line, "%s %*s %*s %*s %s %s %s %s",
             client, frequency, frequencySkew,
             offset, stddev);

          diag(client, offset);

          //float offs = atof(offset);

        /*  if(offs > 1.0)
          {
            diag_error(client);
          } else if(offs > 0.5)
          {
            diag_warn(client);
          }  else
          {
            diag_ok(client);
          }
          diag("Offset", offset);
          diag("Std dev", stddev);
          diag("Freq", frequency);
          diag("Freq skew", frequencySkew);*/
        }
      }
    } else
    {
      diag_warn("system() unavailable");
    }
  } else
  {
    diag_warn("No clients");
  }
}
