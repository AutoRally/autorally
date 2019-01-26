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
 * @file sgd.cu
 * @author Jake Sacks <jsacks6@gatech.edu>
 * @date January 16, 2019
 * @copyright 2019 Georgia Institute of Technology
 * @brief Implementation of the SGDOptimizer class.
 ***********************************************/

SGDOptimizer::SGDOptimizer(int num_params, float lr, float momentum, float dampening, float weight_decay, bool nesterov)
{
  lr_ = lr;
  num_params_ = num_params;
  momentum_ = momentum;
  dampening_ = dampening;
  weight_decay_ = weight_decay;
  nesterov_ = nesterov;

  momentum_buffer_.assign(num_params, 0);
  no_dampen_ = true;
}

SGDOptimizer::~SGDOptimizer()
{}

void SGDOptimizer::slideRunningEstimates(int stride, int param_dim)
{
  int num_steps = int(num_params_ / ((float) param_dim));
  for (int i = 0; i < num_steps - stride; i++) {
    for (int j = 0; j < param_dim; j++) {
      momentum_buffer_[i*param_dim + j] = momentum_buffer_[(i+stride)*param_dim  + j];
    }
  }
  for (int j = 1; j <= stride; j++) {
    for (int i = 0; i < param_dim; i++){
      momentum_buffer_[(num_steps - j)*param_dim + i] = 0;
    }
  }
}

void SGDOptimizer::step(std::vector<float> &params, std::vector<float> &grads)
{
  float grad;

  for (int i = 0; i < num_params_; i++) {
    grad = grads[i];

    if (weight_decay_ != 0) {
      grad += weight_decay_*params[i];
    }

    if (momentum_ != 0) {
      if (no_dampen_) {
        momentum_buffer_[i] = momentum_*momentum_buffer_[i] + grad;
      } else {
        momentum_buffer_[i] = momentum_*momentum_buffer_[i] + (1-dampening_)*grad;
      }

      if (nesterov_) {
        grad += momentum_*momentum_buffer_[i];
      } else {
        grad = momentum_buffer_[i];
      }
    }

    //Update parameter
    params[i] -= lr_ * grad;
  }
  no_dampen_ = false;
}

