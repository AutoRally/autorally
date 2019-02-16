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
 * @file mppi_adaptive_identity_controller.cuh
 * @author Jake Sacks <jsacks6@gatech.edu>
 * @date February 14, 2019
 * @copyright 2019 Georgia Institute of Technology
 * @brief Class definition for the MPPI adaptive identity controller.
 ***********************************************/

#ifndef MPPI_ADAPTIVE_IDENTITY_CONTROLLER_CUH_
#define MPPI_ADAPTIVE_IDENTITY_CONTROLLER_CUH_

#include "autorally_control/path_integral/mppi_adaptive_controller.cuh"
#include <random>

namespace autorally_control{

template <class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS = 2560, int BDIM_X = 64, int BDIM_Y = 1>
class MPPIAdaptiveIdentityController : public MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>
{

public:

  static const int BLOCKSIZE_WRX = 64;
  //NUM_ROLLOUTS has to be divisible by BLOCKSIZE_WRX
  static const int NUM_ROLLOUTS = (ROLLOUTS/BLOCKSIZE_WRX)*BLOCKSIZE_WRX;
  static const int BLOCKSIZE_X = BDIM_X;
  static const int BLOCKSIZE_Y = BDIM_Y;
  static const int STATE_DIM = DYNAMICS_T::STATE_DIM;
  static const int CONTROL_DIM = DYNAMICS_T::CONTROL_DIM;

  //Optimizer settings
  /**
  * @brief Constructor for mppi controller class.
  * @param num_timesteps The number of timesteps to look ahead for.
  * @param dt The time increment. horizon = num_timesteps*dt
  * @param model A basis function model of the system dynamics.
  * @param costs An MppiCosts object.
  * @param mppi_node Handle to a ros node with mppi parameters available as ros params.
  */
  MPPIAdaptiveIdentityController(DYNAMICS_T* model, COSTS_T* costs, OPTIMIZER_T* optim, int num_timesteps, int hz, float gamma,
                     float* exploration_var, float* init_control, int num_optimization_iters = 1,
                     int opt_stride = 1, cudaStream_t stream = 0, std::string dist_type = "normal");

  /**
  * @brief Destructor for mppi controller class.
  */
  ~MPPIAdaptiveIdentityController();

  void slideControlSeq(int stride);

  /**
  * @brief Compute the control given the current state of the system.
  * @param state The current state of the autorally system.
  */
  void computeControl(Eigen::Matrix<float, STATE_DIM, 1> state);

};

#include "mppi_adaptive_identity_controller.cu"

}

#endif /* MPPI_ADAPTIVE_IDENTITY_CONTROLLER_CUH_ */
