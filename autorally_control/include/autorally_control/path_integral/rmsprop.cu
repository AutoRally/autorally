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
 * @file rmsprop.cu
 * @author Jake Sacks <jsacks6@gatech.edu>
 * @date January 20, 2019
 * @copyright 2019 Georgia Institute of Technology
 * @brief Implementation of the RMSpropOptimizer class.
 ***********************************************/

RMSpropOptimizer::RMSpropOptimizer(int num_params, float lr, float alpha, float eps, float weight_decay,
                                    float momentum, bool centered)
{
  lr_ = lr;
  alpha_ = alpha;
  eps_ = eps;
  weight_decay_ = weight_decay;
  momentum_ = momentum;
  centered_ = centered;
  num_params_ = num_params;

  step_.assign(num_params, 0);
  square_avg_.assign(num_params, 0);
  momentum_buffer_.assign(num_params, 0);
  grad_avg_.assign(num_params, 0);
}

RMSpropOptimizer::~RMSpropOptimizer()
{}

void RMSpropOptimizer::resetStep() {
  step_.assign(num_params_, 0);
}

void RMSpropOptimizer::slideRunningEstimates(int stride, int param_dim)
{
  int num_steps = int(num_params_ / ((float) param_dim));
  for (int i = 0; i < num_steps - stride; i++) {
    for (int j = 0; j < param_dim; j++) {
      square_avg_[i*param_dim + j] = square_avg_[(i+stride)*param_dim  + j];
      momentum_buffer_[i*param_dim + j] = momentum_buffer_[(i+stride)*param_dim  + j];
      grad_avg_[i*param_dim + j] = grad_avg_[(i+stride)*param_dim  + j];
    }
  }
  for (int j = 1; j <= stride; j++) {
    for (int i = 0; i < param_dim; i++){
      square_avg_[(num_steps - j)*param_dim + i] = 0;
      momentum_buffer_[(num_steps - j)*param_dim + i] = 0;
      grad_avg_[(num_steps - j)*param_dim + i] = 0;
    }
  }
}

void RMSpropOptimizer::step(std::vector<float> &params, std::vector<float> &grads)
{
  float denom;
  float step_size;
  float grad;

  for (int i = 0; i < num_params_; i++) {
    step_[i] += 1;

    //Decay the first and second moment running average cofficients
    grad = grads[i];
    square_avg_[i] = alpha_*square_avg_[i] + (1-alpha_)*grad*grad;

    //Compute denominator for update
    if (centered_) {
      grad_avg_[i] = alpha_*grad_avg_[i] + (1-alpha_)*grad;
      denom = sqrt(square_avg_[i] - grad_avg_[i]*grad_avg_[i]) + eps_;
    } else {
      denom = sqrt(square_avg_[i]) + eps_;
    }

    //Update parameter
    if (momentum_ > 0.) {
      momentum_buffer_[i] = momentum_*momentum_buffer_[i] + grad/denom;
      params[i] -= lr_ * momentum_buffer_[i];
    } else {
      params[i] -= lr_ * grad/denom;
    }
  }
}

