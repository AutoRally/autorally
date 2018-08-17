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
 * @file param_getter.h
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Function for grabbing parameters from the ROS
 * parameter server.
 ***********************************************/

#ifndef PARAM_GETTER_H_
#define PARAM_GETTER_H_

#include <unistd.h>
#include <string>
#include <ros/ros.h>

namespace autorally_control {

typedef struct
{ 
  bool debug_mode;
  int hz;
  int num_timesteps;
  int num_iters;
  float x_pos;
  float y_pos;
  float heading;
  float gamma;
  float init_steering;
  float init_throttle;
  float steering_std;
  float throttle_std;
  float max_throttle;
  std::string model_path;
} SystemParams;

inline bool fileExists (const std::string& name) {
    return ( access( name.c_str(), F_OK ) != -1 );
}

template <typename T>
T getRosParam(std::string paramName, ros::NodeHandle nh)
{
  std::string key;
  T val;
  bool found = nh.searchParam(paramName, key);
  if (!found){
    ROS_ERROR("Could not find parameter name '%s' in tree of node '%s'", 
              paramName.c_str(), nh.getNamespace().c_str());
  }
  else {
    nh.getParam(key, val);
  }
  return val;
}

void loadParams(SystemParams* params, ros::NodeHandle nh);

}

#endif /*PARAM_GETTER_H_*/


