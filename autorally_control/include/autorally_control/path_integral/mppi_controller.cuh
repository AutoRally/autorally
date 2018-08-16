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
 * @file mppi_controller.cuh
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Class definition for the MPPI controller.
 ***********************************************/

#ifndef MPPI_CONTROLLER_CUH_
#define MPPI_CONTROLLER_CUH_ 

#include "managed.cuh"

#include <autorally_control/ddp/ddp_model_wrapper.h>
#include <autorally_control/ddp/ddp_tracking_costs.h>
#include <autorally_control/ddp/ddp.h>

#include <eigen3/Eigen/Dense>
#include <cuda_runtime.h>
#include <curand.h>

#include "gpu_err_chk.h"

namespace autorally_control{

template <class DYNAMICS_T, class COSTS_T, int ROLLOUTS = 2560, int BDIM_X = 64, int BDIM_Y = 1>
class MPPIController
{

public:

  static const int BLOCKSIZE_WRX = 64;
  //NUM_ROLLOUTS has to be divisible by BLOCKSIZE_WRX
  static const int NUM_ROLLOUTS = (ROLLOUTS/BLOCKSIZE_WRX)*BLOCKSIZE_WRX;
  static const int BLOCKSIZE_X = BDIM_X;
  static const int BLOCKSIZE_Y = BDIM_Y;
  static const int STATE_DIM = DYNAMICS_T::STATE_DIM;
  static const int CONTROL_DIM = DYNAMICS_T::CONTROL_DIM;

  cudaStream_t stream_;

  int numTimesteps_;
  int hz_;
  int optimizationStride_;

  DYNAMICS_T *model_; ///< Model of the autorally system dynamics. 
  COSTS_T *costs_; ///< Autorally system costs.

  //Define DDP optimizer for computing feedback gains around MPPI solution
  ModelWrapperDDP<DYNAMICS_T> *ddp_model_;
  TrackingCostDDP<ModelWrapperDDP<DYNAMICS_T>> *run_cost_;
  TrackingTerminalCost<ModelWrapperDDP<DYNAMICS_T>> *terminal_cost_;
  DDP<ModelWrapperDDP<DYNAMICS_T>> *ddp_solver_;
  typename TrackingCostDDP<ModelWrapperDDP<DYNAMICS_T>>::StateCostWeight Q_;
  typename TrackingTerminalCost<ModelWrapperDDP<DYNAMICS_T>>::Hessian Qf_;
  typename TrackingCostDDP<ModelWrapperDDP<DYNAMICS_T>>::ControlCostWeight R_;
  Eigen::Matrix<float, CONTROL_DIM, 1> U_MIN_;
  Eigen::Matrix<float, CONTROL_DIM, 1> U_MAX_;
  OptimizerResult<ModelWrapperDDP<DYNAMICS_T>> result_;


  /**
  * @brief Constructor for mppi controller class.
  * @param num_timesteps The number of timesteps to look ahead for.
  * @param dt The time increment. horizon = num_timesteps*dt
  * @param model A basis function model of the system dynamics.
  * @param costs An MppiCosts object.
  * @param mppi_node Handle to a ros node with mppi parameters available as ros params.
  */
  MPPIController(DYNAMICS_T* model, COSTS_T* costs, int num_timesteps, int hz, float gamma,
                 float* exploration_var, float* init_control, int num_optimization_iters = 1,
                 int opt_stride = 1, cudaStream_t = 0);

  /**
  * @brief Destructor for mppi controller class.
  */
  ~MPPIController();

  void setCudaStream(cudaStream_t stream);

  /**
  * @brief Allocates cuda memory for all of the controller's device array fields.
  */ 
  void allocateCudaMem();

  /**
  * @brief Frees the cuda memory allocated by allocateCudaMem()
  */
  void deallocateCudaMem();

  void initDDP();
  
  void computeFeedbackGains(Eigen::MatrixXf state);

  OptimizerResult<ModelWrapperDDP<DYNAMICS_T>> getFeedbackGains();

  /*
  * @brief Resets the control commands to there initial values.
  */
  void resetControls();

  void cutThrottle();

  void savitskyGolay();

  void computeNominalTraj(Eigen::Matrix<float, STATE_DIM, 1> state);

  void slideControlSeq(int stride);

  /**
  * @brief Compute the control given the current state of the system.
  * @param state The current state of the autorally system.
  */
  void computeControl(Eigen::Matrix<float, STATE_DIM, 1> state);

  std::vector<float> getControlSeq();

  std::vector<float> getStateSeq();

private:
  int num_iters_;
  float gamma_; ///< Value of the temperature in the softmax.
  float normalizer_; ///< Variable for the normalizing term from sampling.

  curandGenerator_t gen_;

  std::vector<float> traj_costs_; ///< Array of the trajectory costs.
  std::vector<float> state_solution_; ///< Host array for keeping track of the nomimal trajectory.
  std::vector<float> control_solution_;
  std::vector<float> control_hist_;
  std::vector<float> U_;
  std::vector<float> du_; ///< Host array for computing the optimal control update.
  std::vector<float> nu_;
  std::vector<float> init_u_;

  float* state_d_;
  float* nu_d_;
  float* traj_costs_d_;
  float* U_d_;
  float* du_d_;
};

#include "mppi_controller.cu"

}

#endif /* MPPI_CONTROLLER_CUH_ */
