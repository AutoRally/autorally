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
 * @file mppi_adapitve_identity_controller.cu
 * @author Jake Sacks <jsacks6@gatech.edu>
 * @date February 14, 2019
 * @copyright 2019 Georgia Institute of Technology
 * @brief Implementation of the MPPI_ADAPTIVE_IDENTITY_CONTROLLER class.
 ***********************************************/

#define BLOCKSIZE_X MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::BLOCKSIZE_X
#define BLOCKSIZE_Y MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::BLOCKSIZE_Y
#define BLOCKSIZE_WRX MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::BLOCKSIZE_WRX
#define STATE_DIM DYNAMICS_T::STATE_DIM
#define CONTROL_DIM DYNAMICS_T::CONTROL_DIM
#define SHARED_MEM_REQUEST_GRD DYNAMICS_T::SHARED_MEM_REQUEST_GRD
#define SHARED_MEM_REQUEST_BLK DYNAMICS_T::SHARED_MEM_REQUEST_BLK
#define NUM_ROLLOUTS MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::NUM_ROLLOUTS

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
__global__ void varReductionKernel(float* state_costs_d, float normalizer)
{
  int tdx = threadIdx.x;
  int bdx = blockIdx.x;
  if (BLOCKSIZE_X*bdx + tdx < NUM_ROLLOUTS) {
    state_costs_d[BLOCKSIZE_X*bdx + tdx] = (normalizer - state_costs_d[BLOCKSIZE_X*bdx + tdx])/NUM_ROLLOUTS;
  }
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void launchVarReductionKernel(float* costs_d, float normalizer, cudaStream_t stream)
{
  dim3 dimBlock(BLOCKSIZE_X, 1, 1);
  dim3 dimGrid((NUM_ROLLOUTS-1)/BLOCKSIZE_X + 1, 1, 1);
  varReductionKernel<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y><<<dimGrid, dimBlock, 0, stream>>>(
    costs_d, normalizer);
}

#undef BLOCKSIZE_X
#undef BLOCKSIZE_Y
#undef BLOCKSIZE_WRX
#undef STATE_DIM
#undef CONTROL_DIM
#undef SHARED_MEM_REQUEST_GRD
#undef SHARED_MEM_REQUEST_BLK
#undef NUM_ROLLOUTS

/******************************************************************************************************************
MPPI Adaptive Controller implementation
*******************************************************************************************************************/

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
MPPIAdaptiveIdentityController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::MPPIAdaptiveIdentityController(
  DYNAMICS_T* model, COSTS_T* costs, OPTIMIZER_T* optim, int num_timesteps, int hz, float gamma,
  float* exploration_var, float* init_u, int num_optimization_iters, int opt_stride,
  cudaStream_t stream, std::string dist_type)
  : MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>(model, costs, optim,
       num_timesteps, hz, gamma, exploration_var, init_u, num_optimization_iters, opt_stride, stream, dist_type)
{
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
MPPIAdaptiveIdentityController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::~MPPIAdaptiveIdentityController()
{
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIAdaptiveIdentityController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::slideControlSeq(int stride)
{
  typedef MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y> AdaptBase;
  AdaptBase::slideControlSeq(stride);
  AdaptBase::optim_->slideRunningEstimates(stride, CONTROL_DIM);
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIAdaptiveIdentityController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::computeControl(Eigen::Matrix<float, STATE_DIM, 1> state)
{
  typedef MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y> Base;
  typedef MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y> AdaptBase;
  //First transfer the state and current control sequence to the device.
  Base::costs_->paramsToDevice();
  Base::model_->paramsToDevice();

  HANDLE_ERROR( cudaMemcpyAsync(Base::state_d_, state.data(), STATE_DIM*sizeof(float), cudaMemcpyHostToDevice, Base::stream_));
  for (int opt_iter = 0; opt_iter < Base::num_iters_; opt_iter++) {
    HANDLE_ERROR( cudaMemcpyAsync(Base::U_d_, Base::U_.data(), CONTROL_DIM*Base::numTimesteps_*sizeof(float), cudaMemcpyHostToDevice, Base::stream_));
    //Generate a bunch of random numbers
    for (int i=0; i<NUM_ROLLOUTS*Base::numTimesteps_; i++) {
      for (int j=0; j<CONTROL_DIM; j++) {
        if (AdaptBase::dist_type_ == AdaptBase::NORMAL) {
          AdaptBase::noise_[i*CONTROL_DIM + j] = AdaptBase::norm_dist_[j](AdaptBase::rng_);
          //ROS_INFO("du: %f", noise_[i*CONTROL_DIM + j]);
        } else if (AdaptBase::dist_type_ == AdaptBase::LAPLACE) {
          AdaptBase::noise_[i*CONTROL_DIM + j] = AdaptBase::exp_dist_[j](AdaptBase::rng_) - AdaptBase::exp_dist_[j](AdaptBase::rng_);
          //ROS_INFO("du: %f", noise_[i*CONTROL_DIM + j]);
        } else {
          AdaptBase::noise_[i*CONTROL_DIM + j] = AdaptBase::cauchy_dist_[j](AdaptBase::rng_);
        }
      }
    }

    HANDLE_ERROR(cudaMemcpyAsync(Base::du_d_, AdaptBase::noise_.data(),
                                 NUM_ROLLOUTS*Base::numTimesteps_*CONTROL_DIM*sizeof(float),
                                 cudaMemcpyHostToDevice, Base::stream_));

    //Launch the rollout kernel
    launchRolloutKernel<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>(Base::numTimesteps_, Base::state_d_, Base::U_d_,
                                                                       Base::du_d_, Base::nu_d_, Base::traj_costs_d_, Base::model_,
                                                                       Base::costs_, Base::optimizationStride_, Base::stream_);

    HANDLE_ERROR(cudaMemcpyAsync(Base::traj_costs_.data(), Base::traj_costs_d_, NUM_ROLLOUTS*sizeof(float), cudaMemcpyDeviceToHost, Base::stream_));
    //NOTE: The calls to cudaMemcpyAsync are only asynchronous with regards to (1) CPU operations AND (2) GPU operations
    //that are potentially occuring on other streams. Since all the previous kernel/memcpy operations use the same
    //stream, they all occur sequentially with respect to our stream (which is necessary for correct execution)

    //Synchronize stream here since we want to do computations on the CPU
    HANDLE_ERROR( cudaStreamSynchronize(Base::stream_) );

    //Now resume GPU computations
    HANDLE_ERROR(cudaMemcpyAsync(Base::traj_costs_.data(), Base::traj_costs_d_, NUM_ROLLOUTS*sizeof(float), cudaMemcpyDeviceToHost, Base::stream_));
    cudaStreamSynchronize(Base::stream_);

    //Compute the normalizing term
    Base::normalizer_ = 0;
    for (int i = 0; i < NUM_ROLLOUTS; i++) {
      Base::normalizer_ += Base::traj_costs_[i]/NUM_ROLLOUTS;
      //ROS_INFO("COST %f", Base::traj_costs_[i]);
    }
    //ROS_INFO("NORMALIZER: %f, NUM_ROLLOUTS: %d", Base::normalizer_, NUM_ROLLOUTS);

    //Launch variance reduction kernel
    launchVarReductionKernel<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>(Base::traj_costs_d_, Base::normalizer_,
                                                                            Base::stream_);
    HANDLE_ERROR(cudaMemcpyAsync(Base::traj_costs_.data(), Base::traj_costs_d_, NUM_ROLLOUTS*sizeof(float), cudaMemcpyDeviceToHost, Base::stream_));
    cudaStreamSynchronize(Base::stream_);

    //Now resume GPU computations
    HANDLE_ERROR(cudaMemcpyAsync(Base::traj_costs_.data(), Base::traj_costs_d_, NUM_ROLLOUTS*sizeof(float), cudaMemcpyDeviceToHost, Base::stream_));
    cudaStreamSynchronize(Base::stream_);

    //Reset the normalizer such that the reduction kernel below does not modify the weights
    Base::normalizer_ = 1.;

    //Compute the cost weighted average.
     if (AdaptBase::dist_type_ == AdaptBase::NORMAL) {
       launchWeightedReductionNormalKernel<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>(
         Base::traj_costs_d_, Base::U_d_, Base::du_d_, Base::nu_d_, Base::normalizer_, Base::numTimesteps_,
         Base::stream_);
     } else if (AdaptBase::dist_type_ == AdaptBase::LAPLACE) {
       launchWeightedReductionLaplaceKernel<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>(
         Base::traj_costs_d_, Base::U_d_, Base::du_d_, Base::nu_d_, Base::normalizer_, Base::numTimesteps_,
         Base::stream_);
     } else if (AdaptBase::dist_type_ == AdaptBase::CAUCHY) {
       launchWeightedReductionCauchyKernel<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>(
         Base::traj_costs_d_, Base::U_d_, Base::du_d_, Base::nu_d_, Base::normalizer_, Base::numTimesteps_,
         Base::stream_);
     }
    //Transfer control update to host.
    HANDLE_ERROR( cudaMemcpyAsync(Base::du_.data(), Base::du_d_, Base::numTimesteps_*CONTROL_DIM*sizeof(float),
                                  cudaMemcpyDeviceToHost, Base::stream_));
    cudaStreamSynchronize(Base::stream_);

    /*for (int i = 0; i < 5*CONTROL_DIM; i++) {
      ROS_INFO("U[%d]: %f, du[%d]: %f, weight[%d]: %f", i, Base::U_[i], i, Base::du_[i], i, Base::traj_costs_[i]);
    }*/
    AdaptBase::optim_->step(Base::U_, Base::du_);
    /*for (int i = 0; i < 5*CONTROL_DIM; i++) {
      ROS_INFO("U_after[%d]: %f", i, Base::U_[i]);
    }*/
  }

  //Smooth for the next optimization round
  Base::savitskyGolay();
  //Compute the planned trajectory
  Base::computeNominalTraj(state);
}
