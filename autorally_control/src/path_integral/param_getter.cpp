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

void loadParams(SystemParams* params, ros::NodeHandle nh)
{
  params->debug_mode = getRosParam<bool>("debug_mode", nh);
  params->hz = getRosParam<int>("hz", nh);
  params->num_timesteps = getRosParam<int>("num_timesteps", nh);
  params->num_iters = getRosParam<int>("num_iters", nh);
  params->x_pos = getRosParam<double>("x_pos", nh);
  params->y_pos = getRosParam<double>("y_pos", nh);
  params->heading = getRosParam<double>("heading", nh);
  params->gamma = getRosParam<double>("gamma", nh);
  params->init_steering = getRosParam<double>("init_steering", nh);
  params->init_throttle = getRosParam<double>("init_throttle", nh);
  params->steering_std = getRosParam<double>("steering_std", nh);
  params->throttle_std = getRosParam<double>("throttle_std", nh);
  params->max_throttle = getRosParam<double>("max_throttle", nh);
  params->model_path = getRosParam<std::string>("model_path", nh);
  params->optim_type = getRosParam<std::string>("optim_type", nh);
  params->lr = getRosParam<double>("learning_rate", nh);
  params->beta1 = getRosParam<double>("beta1", nh);
  params->beta2 = getRosParam<double>("beta2", nh);
  params->eps = getRosParam<double>("eps", nh);
  params->weight_decay = getRosParam<double>("weight_decay", nh);
  params->amsgrad = getRosParam<bool>("amsgrad", nh);
  params->alpha = getRosParam<double>("alpha", nh);
  params->momentum = getRosParam<double>("momentum", nh);
  params->centered = getRosParam<bool>("centered", nh);
  params->dampening = getRosParam<double>("dampening", nh);
  params->nesterov = getRosParam<bool>("nesterov", nh);
  params->dist_type = getRosParam<std::string>("dist_type", nh);
}

}
