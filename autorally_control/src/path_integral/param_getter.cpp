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
 * @file param_getter.cpp
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Function for grabbing parameters from the ROS
 * parameter server.
 ***********************************************/

#include <autorally_control/path_integral/param_getter.h>

namespace autorally_control {

void loadParams(SystemParams* params, ros::NodeHandle mppi_node)
{
  mppi_node.getParam("debug_mode", params->debug_mode);
  mppi_node.getParam("hz", params->hz);
  mppi_node.getParam("num_timesteps", params->num_timesteps);
  mppi_node.getParam("num_iters", params->num_iters);
  mppi_node.getParam("x_pos", params->x_pos);
  mppi_node.getParam("y_pos", params->y_pos);
  mppi_node.getParam("heading", params->heading);
  mppi_node.getParam("gamma", params->gamma);
  mppi_node.getParam("init_steering", params->init_steering);
  mppi_node.getParam("init_throttle", params->init_throttle);
  mppi_node.getParam("steering_std", params->steering_std);
  mppi_node.getParam("throttle_std", params->throttle_std);
  mppi_node.getParam("max_throttle", params->max_throttle);
  mppi_node.getParam("model_path", params->model_path);
}

}


