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
 * @file adam.cu
 * @author Jake Sacks <jsacks6@gatech.edu>
 * @date January 16, 2019
 * @copyright 2019 Georgia Institute of Technology
 * @brief Implementation of the AdamOptimizer class.
 ***********************************************/

AdamOptimizer::AdamOptimizer(int num_params, float lr, float weight_decay, float beta1, float beta2,
                             float eps, bool amsgrad)
{
  lr_ = lr;
  weight_decay_ = weight_decay;
  beta1_ = beta1;
  beta2_ = beta2;
  eps_ = eps;
  amsgrad_ = amsgrad;
  num_params_ = num_params;

  step_.assign(num_params, 0);
  exp_avg_.assign(num_params, 0);
  exp_avg_sq_.assign(num_params, 0);
  max_exp_avg_sq_.assign(num_params, 0);
}

AdamOptimizer::~AdamOptimizer()
{}

void AdamOptimizer::resetStep() {
  step_.assign(num_params_, 0);
}

void AdamOptimizer::slideRunningEstimates(int stride, int param_dim)
{
  int num_steps = int(num_params_ / ((float) param_dim));
  for (int i = 0; i < num_steps - stride; i++) {
    for (int j = 0; j < param_dim; j++) {
      exp_avg_[i*param_dim + j] = exp_avg_[(i+stride)*param_dim  + j];
      exp_avg_sq_[i*param_dim + j] = exp_avg_sq_[(i+stride)*param_dim  + j];
    }
  }
  for (int j = 1; j <= stride; j++) {
    for (int i = 0; i < param_dim; i++){
      exp_avg_[(num_steps - j)*param_dim + i] = 0;
      exp_avg_sq_[(num_steps - j)*param_dim + i] = 0;
    }
  }
}

void AdamOptimizer::step(std::vector<float> &params, std::vector<float> &grads)
{
  float denom;
  float bias_correction1, bias_correction2;
  float step_size;
  float grad;

  for (int i = 0; i < num_params_; i++) {
    step_[i] += 1;

    //Decay the first and second moment running average cofficients
    grad = grads[i];
    exp_avg_[i] = beta1_ * exp_avg_[i] + (1 - beta1_) * grad;
    exp_avg_sq_[i] = beta2_ * exp_avg_sq_[i] + (1 - beta2_) * grad * grad;

    //Compute denominator for update
    if (amsgrad_) {
      max_exp_avg_sq_[i] = std::max(max_exp_avg_sq_[i], exp_avg_sq_[i]);
      denom = sqrt(max_exp_avg_sq_[i]) + eps_;
    } else {
      denom = sqrt(exp_avg_sq_[i]) + eps_;
    }

    //Compute step size
    bias_correction1 = 1 - pow(beta1_, step_[i]);
    bias_correction2 = 1 - pow(beta2_, step_[i]);
    step_size = lr_ * sqrt(bias_correction2) / bias_correction1;

    //Update parameter
    params[i] -= step_size * exp_avg_[i] / denom;
  }
}

