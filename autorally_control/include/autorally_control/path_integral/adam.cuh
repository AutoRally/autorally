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
 * @file adam.cuh
 * @author Jake Sacks <jsacks6@gatech.edu>
 * @date January 16, 2019
 * @copyright 2019 Georgia Institute of Technology
 * @brief Class definition for the AdamOtimizer.
 ***********************************************/

#ifndef ADAM_OPTIMIZER_CUH_
#define ADAM_OPTIMIZER_CUH_

#include <vector>
#include <algorithm>
#include <math.h>

namespace autorally_control{

class AdamOptimizer
{

public:

/**
* @brief Constructor Adam optimizer class.
* @param lr Learning rate
* @param weight_decay Weight decay (L2 penalty)
* @param beta1 Coefficients used for computing running averages of gradient and its square
* @param beta2
* @param eps Term added ot the denominator to improve numerical stability
* @param amsgrad Flag to use AMSGrad variant of Adam
*/

AdamOptimizer(int num_params, float lr=1e-3, float weight_decay = 0, float beta1=0.9, float beta2=0.999,
              float eps=1e-8, bool amsgrad = false);

/**
* @brief Destructor for Adam optimizer class.
*/
~AdamOptimizer();

void slideRunningEstimates(int stride, int param_dim);
void resetStep();
void step(std::vector<float> &params, std::vector<float> &grads);

private:
  float lr_; ///< learning rate
  float weight_decay_; ///< weight decay (L2 penalty)
  int num_params_; ///< number of parameters to optimize

  float beta1_; ///< coefficients used for computing running averages of gradient and its square
  float beta2_;
  float eps_; ///< term added ot the denominator to improve numerical stability
  bool amsgrad_; ///< flag to use AMSGrad variant of Adam

  std::vector<float> step_;
  std::vector<float> exp_avg_;
  std::vector<float> exp_avg_sq_;
  std::vector<float> max_exp_avg_sq_;
};

#include "adam.cu"

}

#endif /* ADAM_OPTIMIZER_CUH_ */
