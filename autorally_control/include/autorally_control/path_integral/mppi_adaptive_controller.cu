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
 * @file mppi_adapitve_controller.cu
 * @author Jake Sacks <jsacks6@gatech.edu>
 * @date Jan 16, 2019
 * @copyright 2019 Georgia Institute of Technology
 * @brief Implementation of the MPPI_ADAPTIVE_CONTROLLER class.
 ***********************************************/

#define BLOCKSIZE_X MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::BLOCKSIZE_X
#define BLOCKSIZE_Y MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::BLOCKSIZE_Y
#define BLOCKSIZE_WRX MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::BLOCKSIZE_WRX
#define STATE_DIM DYNAMICS_T::STATE_DIM
#define CONTROL_DIM DYNAMICS_T::CONTROL_DIM
#define SHARED_MEM_REQUEST_GRD DYNAMICS_T::SHARED_MEM_REQUEST_GRD
#define SHARED_MEM_REQUEST_BLK DYNAMICS_T::SHARED_MEM_REQUEST_BLK
#define NUM_ROLLOUTS MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::NUM_ROLLOUTS

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
__global__ void rolloutKernelWithState(int num_timesteps, float* state_d, float* state_traj_d, float* U_d, float* du_d,
                                       float* nu_d, float* costs_d, DYNAMICS_T dynamics_model, COSTS_T mppi_costs,
                                       int opt_delay)
{
  int i,j;
  int tdx = threadIdx.x;
  int tdy = threadIdx.y;
  int bdx = blockIdx.x;

  //Initialize the local state, controls, and noise
  float* s;
  float* s_der;
  float* u;
  float* nu;
  float* du;
  int* crash;

  //Create shared arrays for holding state and control data.
  __shared__ float state_shared[BLOCKSIZE_X*STATE_DIM];
  __shared__ float state_der_shared[BLOCKSIZE_X*STATE_DIM];
  __shared__ float control_shared[BLOCKSIZE_X*CONTROL_DIM];
  __shared__ float control_var_shared[BLOCKSIZE_X*CONTROL_DIM];
  __shared__ float exploration_variance[BLOCKSIZE_X*CONTROL_DIM];
  __shared__ int crash_status[BLOCKSIZE_X];
  //Create a shared array for the dynamics model to use
  __shared__ float theta[SHARED_MEM_REQUEST_GRD + SHARED_MEM_REQUEST_BLK*BLOCKSIZE_X];

  //Initialize trajectory cost
  float running_cost = 0;

  //Initialize the dynamics model.
  dynamics_model.cudaInit(theta);

  int global_idx = BLOCKSIZE_X*bdx + tdx;
  if (global_idx < NUM_ROLLOUTS) {
    //Portion of the shared array belonging to each x-thread index.
    s = &state_shared[tdx*STATE_DIM];
    s_der = &state_der_shared[tdx*STATE_DIM];
    u = &control_shared[tdx*CONTROL_DIM];
    du = &control_var_shared[tdx*CONTROL_DIM];
    nu = &exploration_variance[tdx*CONTROL_DIM];
    crash = &crash_status[tdx];
    //Load the initial state, nu, and zero the noise
    for (i = tdy; i < STATE_DIM; i+= blockDim.y) {
      s[i] = state_d[i];
      s_der[i] = 0;
    }
    //Load nu
    for (i = tdy; i < CONTROL_DIM; i+= blockDim.y) {
      u[i] = 0;
      du[i] = 0;
      nu[i] = nu_d[i];
    }
    crash[0] = 0;
  }
  __syncthreads();
  /*<----Start of simulation loop-----> */
  for (i = 0; i < num_timesteps; i++) {
    if (global_idx < NUM_ROLLOUTS) {
      for (j = tdy; j < CONTROL_DIM; j+= blockDim.y) {
        //Noise free rollout
        if (global_idx == 0 || i < opt_delay) { //Don't optimize variables that are already being executed
          du[j] = 0.0;
          u[j] = U_d[i*CONTROL_DIM + j];
        }
        else if (global_idx >= .99*NUM_ROLLOUTS) {
          du[j] = du_d[CONTROL_DIM*num_timesteps*(BLOCKSIZE_X*bdx + tdx) + i*CONTROL_DIM + j];//*nu[j];
          u[j] = du[j];
        }
        else {
          du[j] = du_d[CONTROL_DIM*num_timesteps*(BLOCKSIZE_X*bdx + tdx) + i*CONTROL_DIM + j];//*nu[j];
          u[j] = U_d[i*CONTROL_DIM + j] + du[j];
        }
        du_d[CONTROL_DIM*num_timesteps*(BLOCKSIZE_X*bdx + tdx) + i*CONTROL_DIM + j] = u[j];
      }
      //Save current state
      for (j = tdy; j < STATE_DIM; j += blockDim.y) {
        state_traj_d[STATE_DIM*num_timesteps*(BLOCKSIZE_X*bdx + tdx) + i*STATE_DIM +j] = s[j];
      }
    }
    __syncthreads();
    if (tdy == 0 && global_idx < NUM_ROLLOUTS){
      dynamics_model.enforceConstraints(s, u);
    }
    __syncthreads();
    //Compute the cost of the being in the current state
    if (tdy == 0 && global_idx < NUM_ROLLOUTS && i > 0 && crash[0] > -1) {
      //Running average formula
      running_cost += (mppi_costs.computeCost(s, u, du, nu, crash, i) - running_cost)/(1.0*i);
    }
    //Compute the dynamics
    if (global_idx < NUM_ROLLOUTS){
      dynamics_model.computeStateDeriv(s, u, s_der, theta);
    }
    __syncthreads();
    //Update the state
    if (global_idx < NUM_ROLLOUTS){
      dynamics_model.incrementState(s, s_der);
    }
    //Check to see if the rollout will result in a (physical) crash.
    if (tdy == 0 && global_idx < NUM_ROLLOUTS) {
      mppi_costs.getCrash(s, crash);
    }
  }
  /* <------- End of the simulation loop ----------> */
  if (global_idx < NUM_ROLLOUTS && tdy == 0) {   //Write cost results back to global memory.
    costs_d[(BLOCKSIZE_X)*bdx + tdx] = running_cost + mppi_costs.terminalCost(s);
  }
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
__global__ void weightedReductionNormalKernel(float* states_d, float* U_d, float* du_d, float* nu_d,
                                        float normalizer, int num_timesteps)
{
  int tdx = threadIdx.x;
  int bdx = blockIdx.x;

  __shared__ float u_system[STATE_DIM*((NUM_ROLLOUTS-1)/BLOCKSIZE_WRX + 1)];
  int stride = BLOCKSIZE_WRX;

  float u[CONTROL_DIM];

  int i,j;
  for (i = 0; i < CONTROL_DIM; i++) {
    u[i] = 0;
  }

  for (j = 0; j < CONTROL_DIM; j++) {
    u_system[tdx*CONTROL_DIM + j] = 0;
  }
  __syncthreads();

  if (BLOCKSIZE_WRX*tdx < NUM_ROLLOUTS) {
    float weight = 0;
    for (i = 0; i < stride; i++) {
      if (stride*tdx + i < NUM_ROLLOUTS) {
        weight = states_d[stride*tdx + i]/normalizer;
        for (j = 0; j < CONTROL_DIM; j++) {
          u[j] = U_d[bdx*CONTROL_DIM + j];
          u[j] -= du_d[(stride*tdx + i)*(num_timesteps*CONTROL_DIM) + bdx*CONTROL_DIM + j];
          u_system[tdx*CONTROL_DIM + j] += weight*u[j];
        }
      }
    }
  }
  __syncthreads();
  if (tdx == 0 && bdx < num_timesteps) {
    for (i = 0; i < CONTROL_DIM; i++) {
      u[i] = 0;
    }
    for (i = 0; i < (NUM_ROLLOUTS-1)/BLOCKSIZE_WRX + 1; i++) {
      for (j = 0; j < CONTROL_DIM; j++) {
        u[j] += u_system[CONTROL_DIM*i + j];
      }
    }
    for (i = 0; i < CONTROL_DIM; i++) {
      du_d[CONTROL_DIM*bdx + i] = u[i];
    }
  }
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
__global__ void weightedReductionLaplaceKernel(float* states_d, float* U_d, float* du_d, float* nu_d,
                                               float normalizer, int num_timesteps)
{
  int tdx = threadIdx.x;
  int bdx = blockIdx.x;

  __shared__ float u_system[STATE_DIM*((NUM_ROLLOUTS-1)/BLOCKSIZE_WRX + 1)];
  int stride = BLOCKSIZE_WRX;

  float u[CONTROL_DIM];
  float du[CONTROL_DIM];

  int i,j;
  for (i = 0; i < CONTROL_DIM; i++) {
    u[i] = 0;
  }

  for (j = 0; j < CONTROL_DIM; j++) {
    u_system[tdx*CONTROL_DIM + j] = 0;
  }
  __syncthreads();

  float sign;
  if (BLOCKSIZE_WRX*tdx < NUM_ROLLOUTS) {
    float weight = 0;
    for (i = 0; i < stride; i++) {
      if (stride*tdx + i < NUM_ROLLOUTS) {
        weight = states_d[stride*tdx + i]/normalizer;
        for (j = 0; j < CONTROL_DIM; j++) {
          du[j] = du_d[(stride*tdx + i)*(num_timesteps*CONTROL_DIM) + bdx*CONTROL_DIM + j];
          u[j] =  U_d[bdx*CONTROL_DIM + j] - du[j];
          sign = u[j]>0 ? 1. : -1.;
          if (u[j] == 0) sign = 0.;
          u_system[tdx*CONTROL_DIM + j] += weight * nu_d[j] * sign;
        }
      }
    }
  }
  __syncthreads();
  if (tdx == 0 && bdx < num_timesteps) {
    for (i = 0; i < CONTROL_DIM; i++) {
      u[i] = 0;
    }
    for (i = 0; i < (NUM_ROLLOUTS-1)/BLOCKSIZE_WRX + 1; i++) {
      for (j = 0; j < CONTROL_DIM; j++) {
        u[j] += u_system[CONTROL_DIM*i + j];
      }
    }
    for (i = 0; i < CONTROL_DIM; i++) {
      du_d[CONTROL_DIM*bdx + i] = u[i];
    }
  }
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
__global__ void weightedReductionCauchyKernel(float* states_d, float* U_d, float* du_d, float* nu_d,
                                              float normalizer, int num_timesteps)
{
  int tdx = threadIdx.x;
  int bdx = blockIdx.x;

  __shared__ float u_system[STATE_DIM*((NUM_ROLLOUTS-1)/BLOCKSIZE_WRX + 1)];
  int stride = BLOCKSIZE_WRX;

  float u[CONTROL_DIM];

  int i,j;
  for (i = 0; i < CONTROL_DIM; i++) {
    u[i] = 0;
  }

  for (j = 0; j < CONTROL_DIM; j++) {
    u_system[tdx*CONTROL_DIM + j] = 0;
  }
  __syncthreads();

  float sign;
  if (BLOCKSIZE_WRX*tdx < NUM_ROLLOUTS) {
    float weight = 0;
    for (i = 0; i < stride; i++) {
      if (stride*tdx + i < NUM_ROLLOUTS) {
        weight = states_d[stride*tdx + i]/normalizer;
        for (j = 0; j < CONTROL_DIM; j++) {
          u[j] = du_d[(stride*tdx + i)*(num_timesteps*CONTROL_DIM) + bdx*CONTROL_DIM + j];
          u[j] = U_d[bdx*CONTROL_DIM + j] - u[j];
          u_system[tdx*CONTROL_DIM + j] += weight * 4 * u[j] / (1 + pow(u[j]/nu_d[j], 2));
        }
      }
    }
  }
  __syncthreads();
  if (tdx == 0 && bdx < num_timesteps) {
    for (i = 0; i < CONTROL_DIM; i++) {
      u[i] = 0;
    }
    for (i = 0; i < (NUM_ROLLOUTS-1)/BLOCKSIZE_WRX + 1; i++) {
      for (j = 0; j < CONTROL_DIM; j++) {
        u[j] += u_system[CONTROL_DIM*i + j];
      }
    }
    for (i = 0; i < CONTROL_DIM; i++) {
      du_d[CONTROL_DIM*bdx + i] = u[i];
    }
  }
}


template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void launchRolloutKernelWithState(int num_timesteps, float* state_d, float* state_traj_d, float* U_d, float* du_d,
                                  float* nu_d, float* costs_d, DYNAMICS_T *dynamics_model, COSTS_T *mppi_costs,
                                  int opt_delay, cudaStream_t stream)
{
  const int GRIDSIZE_X = (NUM_ROLLOUTS-1)/BLOCKSIZE_X + 1;
  dim3 dimBlock(BLOCKSIZE_X, BLOCKSIZE_Y, 1);
  dim3 dimGrid(GRIDSIZE_X, 1, 1);
  rolloutKernelWithState<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y><<<dimGrid, dimBlock, 0, stream>>>(num_timesteps, state_d, state_traj_d,
    U_d, du_d, nu_d, costs_d, *dynamics_model, *mppi_costs, opt_delay);
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void launchWeightedReductionNormalKernel(float* state_costs_d, float* U_d, float* du_d, float* nu_d,
                                   float normalizer, int num_timesteps, cudaStream_t stream)
{
  dim3 dimBlock((NUM_ROLLOUTS-1)/BLOCKSIZE_WRX + 1, 1, 1);
  dim3 dimGrid(num_timesteps, 1, 1);
  weightedReductionNormalKernel<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y><<<dimGrid, dimBlock, 0,
  stream>>> (state_costs_d, U_d, du_d, nu_d, normalizer, num_timesteps);
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void launchWeightedReductionLaplaceKernel(float* state_costs_d, float* U_d, float* du_d, float* nu_d,
                                   float normalizer, int num_timesteps, cudaStream_t stream)
{
  dim3 dimBlock((NUM_ROLLOUTS-1)/BLOCKSIZE_WRX + 1, 1, 1);
  dim3 dimGrid(num_timesteps, 1, 1);
  weightedReductionLaplaceKernel<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y><<<dimGrid, dimBlock, 0, stream>>>
    (state_costs_d, U_d, du_d, nu_d, normalizer, num_timesteps);
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void launchWeightedReductionCauchyKernel(float* state_costs_d, float* U_d, float* du_d, float* nu_d,
                                          float normalizer, int num_timesteps, cudaStream_t stream)
{
  dim3 dimBlock((NUM_ROLLOUTS-1)/BLOCKSIZE_WRX + 1, 1, 1);
  dim3 dimGrid(num_timesteps, 1, 1);
  weightedReductionCauchyKernel<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y><<<dimGrid, dimBlock, 0, stream>>>
    (state_costs_d, U_d, du_d, nu_d, normalizer, num_timesteps);
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
MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::MPPIAdaptiveController(
  DYNAMICS_T* model, COSTS_T* costs, OPTIMIZER_T* optim, int num_timesteps, int hz, float gamma,
  float* exploration_var, float* init_u, int num_optimization_iters, int opt_stride,
  cudaStream_t stream, std::string dist_type, bool use_cem)
  : MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>(model, costs, num_timesteps, hz, gamma,
       exploration_var, init_u, num_optimization_iters, opt_stride, stream), rng_(rd_())
{
  //Set the optimizer
  optim_ = optim;

  use_cem_ = use_cem;

  //Initialize vectors
  typedef MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y> Base;
  std_.assign(exploration_var, exploration_var + CONTROL_DIM);
  grads_.assign(Base::numTimesteps_*CONTROL_DIM, 0);
  noise_.assign(NUM_ROLLOUTS*Base::numTimesteps_*CONTROL_DIM, 0);

  //Set distribution type
  if (dist_type == "normal") {
    dist_type_ = NORMAL;
    for (int i=0; i<CONTROL_DIM; i++)
      norm_dist_.push_back(std::normal_distribution<float>(0., exploration_var[i]));
  } else if (dist_type == "laplace") {
    dist_type_ = LAPLACE;
    for (int i=0; i<CONTROL_DIM; i++) {
      exp_dist_.push_back(std::exponential_distribution<float>(1. / exploration_var[i]));
    }
  } else {
    dist_type_ = CAUCHY;
    for (int i=0; i<CONTROL_DIM; i++) {
      cauchy_dist_.push_back(std::cauchy_distribution<float>(0., exploration_var[i]));
    }
  }

  state_traj_.assign(NUM_ROLLOUTS*Base::numTimesteps_*STATE_DIM, 0);
  HANDLE_ERROR( cudaMalloc((void**)&state_traj_d_, NUM_ROLLOUTS*Base::numTimesteps_*STATE_DIM*sizeof(float)));
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::~MPPIAdaptiveController()
{
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::slideControlSeq(int stride)
{
  MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::slideControlSeq(stride);
  optim_->slideRunningEstimates(stride, CONTROL_DIM);
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::computeControl(Eigen::Matrix<float, STATE_DIM, 1> state)
{
  typedef MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y> Base;
  //First transfer the state and current control sequence to the device.
  Base::costs_->paramsToDevice();
  Base::model_->paramsToDevice();

  HANDLE_ERROR( cudaMemcpyAsync(Base::state_d_, state.data(), STATE_DIM*sizeof(float), cudaMemcpyHostToDevice, Base::stream_));
  for (int opt_iter = 0; opt_iter < Base::num_iters_; opt_iter++) {
    HANDLE_ERROR( cudaMemcpyAsync(Base::U_d_, Base::U_.data(), CONTROL_DIM*Base::numTimesteps_*sizeof(float), cudaMemcpyHostToDevice, Base::stream_));
    //Generate a bunch of random numbers
    for (int i=0; i<NUM_ROLLOUTS*Base::numTimesteps_; i++) {
      for (int j=0; j<CONTROL_DIM; j++) {
        if (dist_type_ == NORMAL) {
          noise_[i*CONTROL_DIM + j] = norm_dist_[j](rng_);
          //ROS_INFO("du: %f", noise_[i*CONTROL_DIM + j]);
        } else if (dist_type_ == LAPLACE) {
          noise_[i*CONTROL_DIM + j] = exp_dist_[j](rng_) - exp_dist_[j](rng_);
          //ROS_INFO("du: %f", noise_[i*CONTROL_DIM + j]);
        } else {
          noise_[i*CONTROL_DIM + j] = cauchy_dist_[j](rng_);
        }
      }
    }

    HANDLE_ERROR(cudaMemcpyAsync(Base::du_d_, noise_.data(),
                                 NUM_ROLLOUTS*Base::numTimesteps_*CONTROL_DIM*sizeof(float),
                                 cudaMemcpyHostToDevice, Base::stream_));

    //curandGenerateNormal(Base::gen_, Base::du_d_, NUM_ROLLOUTS*Base::numTimesteps_*CONTROL_DIM, 0.0, 1.0);

    //Launch the rollout kernel
    launchRolloutKernelWithState<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>(Base::numTimesteps_, Base::state_d_, state_traj_d_, Base::U_d_,
                                                                       Base::du_d_, Base::nu_d_, Base::traj_costs_d_, Base::model_,
                                                                       Base::costs_, Base::optimizationStride_, Base::stream_);

    HANDLE_ERROR(cudaMemcpyAsync(state_traj_.data(), state_traj_d_, NUM_ROLLOUTS*Base::numTimesteps_*STATE_DIM*sizeof(float), cudaMemcpyDeviceToHost, Base::stream_));
    HANDLE_ERROR(cudaMemcpyAsync(Base::traj_costs_.data(), Base::traj_costs_d_, NUM_ROLLOUTS*sizeof(float), cudaMemcpyDeviceToHost, Base::stream_));
    //NOTE: The calls to cudaMemcpyAsync are only asynchronous with regards to (1) CPU operations AND (2) GPU operations
    //that are potentially occuring on other streams. Since all the previous kernel/memcpy operations use the same
    //stream, they all occur sequentially with respect to our stream (which is necessary for correct execution)

    //Synchronize stream here since we want to do computations on the CPU
    HANDLE_ERROR( cudaStreamSynchronize(Base::stream_) );

    if (use_cem_) {
      std::vector<float> traj_costs_tmp(Base::traj_costs_);
      //std::sort(&traj_costs_tmp[0], &traj_costs_tmp[NUM_ROLLOUTS-1]);
      std::sort(traj_costs_tmp.begin(), traj_costs_tmp.begin() + NUM_ROLLOUTS);
      int num_elites = static_cast<int>(Base::gamma_*NUM_ROLLOUTS);
      float cutoff = traj_costs_tmp[num_elites];
      for (int i = 0; i < NUM_ROLLOUTS; i++) {
        Base::traj_costs_[i] = (Base::traj_costs_[i] <= cutoff) ? 1.f : 0.f;
      }
      HANDLE_ERROR(cudaMemcpyAsync(Base::traj_costs_d_, Base::traj_costs_.data(),
                  NUM_ROLLOUTS*sizeof(float), cudaMemcpyHostToDevice, Base::stream_));
    } else {
      //Compute the baseline (minimum) sampled cost
      float baseline = Base::traj_costs_[0];
      for (int i = 0; i < NUM_ROLLOUTS; i++) {
        if (Base::traj_costs_[i] < baseline){
          baseline = Base::traj_costs_[i];
        }
      }

      //Now resume GPU computations
      launchNormExpKernel<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>(Base::traj_costs_d_, Base::gamma_, baseline, Base::stream_);
      HANDLE_ERROR(cudaMemcpyAsync(Base::traj_costs_.data(), Base::traj_costs_d_,
                  NUM_ROLLOUTS*sizeof(float), cudaMemcpyDeviceToHost, Base::stream_));
    }

    cudaStreamSynchronize(Base::stream_);

    //Compute the normalizing term
    Base::normalizer_ = 0;
    for (int i = 0; i < NUM_ROLLOUTS; i++) {
      Base::normalizer_ += Base::traj_costs_[i];
    }

    //Compute the cost weighted average.
     if (dist_type_ == NORMAL) {
       launchWeightedReductionNormalKernel<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>(
         Base::traj_costs_d_, Base::U_d_, Base::du_d_, Base::nu_d_, Base::normalizer_, Base::numTimesteps_,
         Base::stream_);
     } else if (dist_type_ == LAPLACE) {
       launchWeightedReductionLaplaceKernel<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>(
         Base::traj_costs_d_, Base::U_d_, Base::du_d_, Base::nu_d_, Base::normalizer_, Base::numTimesteps_,
         Base::stream_);
     } else if (dist_type_ == CAUCHY) {
       launchWeightedReductionCauchyKernel<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>(
         Base::traj_costs_d_, Base::U_d_, Base::du_d_, Base::nu_d_, Base::normalizer_, Base::numTimesteps_,
         Base::stream_);
     }
    //Transfer control update to host.
    HANDLE_ERROR( cudaMemcpyAsync(Base::du_.data(), Base::du_d_, Base::numTimesteps_*CONTROL_DIM*sizeof(float),
                                  cudaMemcpyDeviceToHost, Base::stream_));
    cudaStreamSynchronize(Base::stream_);

    optim_->step(Base::U_, Base::du_);
  }
  //Smooth for the next optimization round
  Base::savitskyGolay();
  //Compute the planned trajectory
  Base::computeNominalTraj(state);
}

template<class DYNAMICS_T, class COSTS_T, class OPTIMIZER_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
std::vector<float> MPPIAdaptiveController<DYNAMICS_T, COSTS_T, OPTIMIZER_T, ROLLOUTS, BDIM_X, BDIM_Y>::getStateTrajectories() {
  return state_traj_;
};
