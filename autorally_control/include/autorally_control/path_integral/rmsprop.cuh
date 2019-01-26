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
 * @file rmsprop.cuh
 * @author Jake Sacks <jsacks6@gatech.edu>
 * @date January 20, 2019
 * @copyright 2019 Georgia Institute of Technology
 * @brief Class definition for the RMSpropOptimizer.
 ***********************************************/

#ifndef RMSPROP_OPTIMIZER_CUH_
#define RMSPROP_OPTIMIZER_CUH_

#include <vector>
#include <algorithm>
#include <math.h>

namespace autorally_control{

class RMSpropOptimizer
{

public:

/**
* @brief Constructor RMSprop optimizer class.
* @param lr Learning rate
* @param alpha Smoothing constant
* @param eps Term added ot the denominator to improve numerical stability
* @param weight_decay Weight decay (L2 penalty)
* @param momentum Momentum factor
* @param centered Flag to compute the centered RMSprop, which normalized the gradient by estimation of its variance
*/

RMSpropOptimizer(int num_params, float lr=1e-3, float alpha=0.99, float eps=1e-8, float weight_decay=0,
                 float momentum=0., bool centered=false);

/**
* @brief Destructor for RMSprop optimizer class.
*/
~RMSpropOptimizer();

void slideRunningEstimates(int stride, int param_dim);
void resetStep();
void step(std::vector<float> &params, std::vector<float> &grads);

private:
  float lr_; ///< learning rate
  float weight_decay_; ///< weight decay (L2 penalty)
  float momentum_; ///< momentum factor
  int num_params_; ///< number of parameters to optimize

  float alpha_; ///< smoothing constant
  float eps_; ///< term added ot the denominator to improve numerical stability
  bool centered_; ///< flag to compute the centered RMSprop

  std::vector<float> step_;
  std::vector<float> square_avg_;
  std::vector<float> momentum_buffer_;
  std::vector<float> grad_avg_;
};

#include "rmsprop.cu"

}

#endif /* RMSPROP_OPTIMIZER_CUH_ */
