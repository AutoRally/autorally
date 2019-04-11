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
 * @file mppi_controller.cu
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Implementation of the mppi_controller class.
 ***********************************************/

/******************************************************************************
//MPPI Kernel Implementations and helper launch files
*******************************************************************************/

#define BLOCKSIZE_X MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::BLOCKSIZE_X
#define BLOCKSIZE_Y MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::BLOCKSIZE_Y
#define BLOCKSIZE_WRX MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::BLOCKSIZE_WRX
#define STATE_DIM DYNAMICS_T::STATE_DIM
#define CONTROL_DIM DYNAMICS_T::CONTROL_DIM
#define SHARED_MEM_REQUEST_GRD DYNAMICS_T::SHARED_MEM_REQUEST_GRD
#define SHARED_MEM_REQUEST_BLK DYNAMICS_T::SHARED_MEM_REQUEST_BLK
#define NUM_ROLLOUTS MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::NUM_ROLLOUTS

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
__global__ void rolloutKernel(int num_timesteps, float* state_d, float* U_d, float* du_d, float* nu_d, 
                              float* costs_d, DYNAMICS_T dynamics_model, COSTS_T mppi_costs, 
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
          du[j] = du_d[CONTROL_DIM*num_timesteps*(BLOCKSIZE_X*bdx + tdx) + i*CONTROL_DIM + j]*nu[j];
          u[j] = du[j];
        }
        else {
          du[j] = du_d[CONTROL_DIM*num_timesteps*(BLOCKSIZE_X*bdx + tdx) + i*CONTROL_DIM + j]*nu[j];
          u[j] = U_d[i*CONTROL_DIM + j] + du[j];
        }
        du_d[CONTROL_DIM*num_timesteps*(BLOCKSIZE_X*bdx + tdx) + i*CONTROL_DIM + j] = u[j];
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

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
__global__ void normExpKernel(float* state_costs_d, float gamma, float baseline)
{
  int tdx = threadIdx.x;
  int bdx = blockIdx.x;
  if (BLOCKSIZE_X*bdx + tdx < NUM_ROLLOUTS) {
    float cost2go = 0;
    cost2go = state_costs_d[BLOCKSIZE_X*bdx + tdx] - baseline;
    state_costs_d[BLOCKSIZE_X*bdx + tdx] = exp(-gamma*cost2go);
  }
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
__global__ void weightedReductionKernel(float* states_d, float* du_d, float* nu_d, 
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
          u[j] = du_d[(stride*tdx + i)*(num_timesteps*CONTROL_DIM) + bdx*CONTROL_DIM + j];
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

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void launchRolloutKernel(int num_timesteps, float* state_d, float* U_d, float* du_d, float* nu_d, 
                         float* costs_d, DYNAMICS_T *dynamics_model, COSTS_T *mppi_costs, 
                         int opt_delay, cudaStream_t stream)
{
  const int GRIDSIZE_X = (NUM_ROLLOUTS-1)/BLOCKSIZE_X + 1;
  //transferMemToConst(dynamics_model.theta_d_);
  dim3 dimBlock(BLOCKSIZE_X, BLOCKSIZE_Y, 1);
  dim3 dimGrid(GRIDSIZE_X, 1, 1);
  //printf("%d \n", sizeof(DYNAMICS_T));
  //int dev;
  //cudaGetDevice(&dev);
  //printf("Device: %d \n", dev);
  //HANDLE_ERROR(cudaMemPrefetchAsync(dynamics_model, sizeof(DYNAMICS_T), dev, stream) );
  //HANDLE_ERROR(cudaMemPrefetchAsync(dynamics_model->control_rngs_d_, 2*sizeof(float2), dev, stream) );
  rolloutKernel<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y><<<dimGrid, dimBlock, 0, stream>>>(num_timesteps, state_d, U_d, 
    du_d, nu_d, costs_d, *dynamics_model, *mppi_costs, opt_delay);
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void launchNormExpKernel(float* costs_d, float gamma, float baseline, cudaStream_t stream)
{
  dim3 dimBlock(BLOCKSIZE_X, 1, 1);
  dim3 dimGrid((NUM_ROLLOUTS-1)/BLOCKSIZE_X + 1, 1, 1);
  normExpKernel<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y><<<dimGrid, dimBlock, 0, stream>>>(costs_d, gamma, baseline);
}

//Launches the multiplication and reduction kernel
template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void launchWeightedReductionKernel(float* state_costs_d, float* du_d, float* nu_d, 
                                  float normalizer, int num_timesteps, cudaStream_t stream)
{
    dim3 dimBlock((NUM_ROLLOUTS-1)/BLOCKSIZE_WRX + 1, 1, 1);
    dim3 dimGrid(num_timesteps, 1, 1);
    weightedReductionKernel<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y><<<dimGrid, dimBlock, 0, stream>>>(state_costs_d, du_d, nu_d, normalizer, num_timesteps);
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
MPPI Controller implementation
*******************************************************************************************************************/

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::MPPIController(DYNAMICS_T* model, COSTS_T* costs, 
                                                                              int num_timesteps, int hz, float gamma, 
                                                                              float* exploration_var, float* init_u, 
                                                                              int num_optimization_iters, int opt_stride,
                                                                              cudaStream_t stream)
{
  //Initialize internal classes which use the CUDA API.
  model_ = model;
  costs_ = costs;
  curandCreateGenerator(&gen_, CURAND_RNG_PSEUDO_DEFAULT);
  curandSetPseudoRandomGeneratorSeed(gen_, 1234ULL);

  //Set the CUDA stream, and attach unified memory to the particular stream.
  //This must be done AFTER all internal classes that use unified memory are initialized (cost and model)
  setCudaStream(stream);

  //Initialize parameters, including the number of rollouts and timesteps
  hz_ = hz;
  numTimesteps_ = num_timesteps;
  optimizationStride_ = opt_stride;
  gamma_ = gamma;
  num_iters_ = num_optimization_iters;

  //Initialize host vectors
  nu_.assign(exploration_var, exploration_var + CONTROL_DIM);
  init_u_.assign(init_u, init_u + CONTROL_DIM);
  control_hist_.assign(2*CONTROL_DIM, 0);
  state_solution_.assign(numTimesteps_*STATE_DIM, 0);
  control_solution_.assign(numTimesteps_*CONTROL_DIM, 0);
  du_.resize(numTimesteps_*CONTROL_DIM);
  U_.resize(numTimesteps_*CONTROL_DIM);
  traj_costs_.resize(NUM_ROLLOUTS);

  //Allocate memory on the device.
  allocateCudaMem();
  //Transfer exploration variance to device.
  HANDLE_ERROR(cudaMemcpyAsync(nu_d_, nu_.data(), CONTROL_DIM*sizeof(float), cudaMemcpyHostToDevice, stream_));
  //Get the parameters for the control input and initialize the sequence.
  initDDP();
  resetControls();
  //Make sure all cuda operations have finished.
  cudaStreamSynchronize(stream_);
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::~MPPIController()
{}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::setCudaStream(cudaStream_t stream)
{
  //Set the CUDA stream and attach unified memory in object to that stream
  stream_ = stream;
  model_->bindToStream(stream_);
  costs_->bindToStream(stream_);
  curandSetStream(gen_, stream_);
} 

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::allocateCudaMem()
{
  HANDLE_ERROR( cudaMalloc((void**)&state_d_, STATE_DIM*sizeof(float)));
  HANDLE_ERROR( cudaMalloc((void**)&nu_d_, STATE_DIM*sizeof(float)));
  HANDLE_ERROR( cudaMalloc((void**)&traj_costs_d_, NUM_ROLLOUTS*sizeof(float)));
  HANDLE_ERROR( cudaMalloc((void**)&U_d_, CONTROL_DIM*numTimesteps_*sizeof(float)));
  HANDLE_ERROR( cudaMalloc((void**)&du_d_, NUM_ROLLOUTS*numTimesteps_*CONTROL_DIM*sizeof(float)));
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::deallocateCudaMem(){
  cudaFree(state_d_);
  cudaFree(nu_d_);
  cudaFree(traj_costs_d_);
  cudaFree(U_d_);
  cudaFree(du_d_);
  //Free cuda memory used by the model and costs.
  model_->freeCudaMem();
  costs_->freeCudaMem();
  cudaStreamDestroy(stream_);
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::initDDP()
{
  util::DefaultLogger logger;
  bool verbose = false;
  ddp_model_ = new ModelWrapperDDP<DYNAMICS_T>(model_);
  ddp_solver_ = new DDP<ModelWrapperDDP<DYNAMICS_T>>(1.0/hz_, numTimesteps_, 1, &logger, verbose);

  Q_.setIdentity();
  Q_.diagonal() << 0.5, 0.5, 0.25, 0.0, 0.05, 0.01, 0.01;

  Qf_.setIdentity();
  Qf_.diagonal() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  R_.setIdentity();
  R_.diagonal() << 10.0, 10.0;

  U_MIN_ << model_->control_rngs_[0].x, model_->control_rngs_[1].x;
  U_MAX_ << model_->control_rngs_[0].y, model_->control_rngs_[1].y;
  
  //Define the running and terminal cost
  run_cost_ = new TrackingCostDDP<ModelWrapperDDP<DYNAMICS_T>>(Q_, R_, numTimesteps_);
  terminal_cost_ = new TrackingTerminalCost<ModelWrapperDDP<DYNAMICS_T>>(Qf_);
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::computeFeedbackGains(Eigen::MatrixXf state)
{
  Eigen::MatrixXf control_traj = Eigen::MatrixXf::Zero(CONTROL_DIM, numTimesteps_);
  for (int t = 0; t < numTimesteps_; t++){
    for (int i = 0; i < CONTROL_DIM; i++){
      control_traj(i,t) = control_solution_[CONTROL_DIM*t + i];
    }
  }
  run_cost_->setTargets(state_solution_.data(), control_solution_.data(), numTimesteps_);
  terminal_cost_->xf = run_cost_->traj_target_x_.col(numTimesteps_ - 1);
  result_ = ddp_solver_->run(state, control_traj, *ddp_model_, *run_cost_, *terminal_cost_, U_MIN_, U_MAX_);
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
OptimizerResult<ModelWrapperDDP<DYNAMICS_T>> MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::getFeedbackGains()
{
  return result_;
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::resetControls()
{
  int i,j;
  //Set all the control values to their initial settings.
  for (i = 0; i < numTimesteps_; i++) {
    for (j = 0; j < CONTROL_DIM; j++) {
      U_[i*CONTROL_DIM + j] = init_u_[j];
    }
  }
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::cutThrottle()
{
  costs_->params_.desired_speed = 0.0;
  model_->control_rngs_[1].y = 0.0; //Max throttle to zero
  costs_->paramsToDevice();
  model_->paramsToDevice();
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::savitskyGolay()
{
  int i,j;
  Eigen::MatrixXf filter(1,5);
  Eigen::MatrixXf U_smoothed = Eigen::MatrixXf::Zero(8 + numTimesteps_, CONTROL_DIM);

  filter << -3, 12, 17, 12, -3;
  filter /= 35.0;
  for (i = 0; i < numTimesteps_ + 4; i++){
    if (i < 2) {
      for (j = 0; j < CONTROL_DIM; j++){
        U_smoothed(i, j) = control_hist_[CONTROL_DIM*i + j];
      }
    }
    else if (i < numTimesteps_ + 2) {
      for (j = 0; j < CONTROL_DIM; j++){
        U_smoothed(i,j) = U_[CONTROL_DIM*(i - 2) + j];
      }
    }
    else{
      for (j = 0; j < CONTROL_DIM; j++) {
        U_smoothed(i, j) = U_[CONTROL_DIM*(numTimesteps_ - 1) + j];
      }
    }
  }
  for (i = 0; i < numTimesteps_; i++){
    for (j = 0; j < CONTROL_DIM; j++){
      U_[CONTROL_DIM*i + j] = (filter*U_smoothed.block<5,1>(i,j))(0,0);
    }
  }
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::computeNominalTraj(Eigen::Matrix<float, STATE_DIM, 1> state)
{
  int i,j;
  Eigen::MatrixXf s(7,1);
  Eigen::MatrixXf u(2,1);
  s = state;
  for (i = 0; i < numTimesteps_; i++){
    for (j = 0; j < STATE_DIM; j++){
      //Set the current state solution
      state_solution_[i*STATE_DIM + j] = s(j);
    }
    u << U_[2*i], U_[2*i + 1];
    model_->updateState(s,u);
    //Set current control solution after clamping
    control_solution_[2*i] = u(0);
    control_solution_[2*i + 1] = u(1);
  }
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::slideControlSeq(int stride)
{
  //Slide the control sequence down by stride
  if (stride == 1){
    control_hist_[0] = control_hist_[2];
    control_hist_[1] = control_hist_[3];
    control_hist_[2] = U_[0];
    control_hist_[3] = U_[1];
  }
  else{
    int t = stride - 2;
    for (int i = 0; i < 4; i++){
      control_hist_[i] = U_[t + i];
    }
  }

  for (int i = 0; i < numTimesteps_- stride; i++) {
    for (int j = 0; j < CONTROL_DIM; j++) {
      U_[i*CONTROL_DIM + j] = U_[(i+stride)*CONTROL_DIM + j];
    }
  }
  //Initialize new controls to the init_u_ variable.
  for (int j = 1; j <= stride; j++) {
    for (int i = 0; i < CONTROL_DIM; i++){
      U_[(numTimesteps_ - j)*CONTROL_DIM + i] = init_u_[i];
    }
  }
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
void MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::computeControl(Eigen::Matrix<float, STATE_DIM, 1> state)
{
  //First transfer the state and current control sequence to the device.
  costs_->paramsToDevice();
  model_->paramsToDevice();

  HANDLE_ERROR( cudaMemcpyAsync(state_d_, state.data(), STATE_DIM*sizeof(float), cudaMemcpyHostToDevice, stream_));
  for (int opt_iter = 0; opt_iter < num_iters_; opt_iter++) {
    HANDLE_ERROR( cudaMemcpyAsync(U_d_, U_.data(), CONTROL_DIM*numTimesteps_*sizeof(float), cudaMemcpyHostToDevice, stream_));    
    //Generate a bunch of random numbers
    curandGenerateNormal(gen_, du_d_, NUM_ROLLOUTS*numTimesteps_*CONTROL_DIM, 0.0, 1.0);
    //Launch the rollout kernel
    launchRolloutKernel<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>(numTimesteps_, state_d_, U_d_, du_d_, nu_d_, traj_costs_d_, model_, 
                        costs_, optimizationStride_, stream_);
    HANDLE_ERROR(cudaMemcpyAsync(traj_costs_.data(), traj_costs_d_, NUM_ROLLOUTS*sizeof(float), cudaMemcpyDeviceToHost, stream_));
    //NOTE: The calls to cudaMemcpyAsync are only asynchronous with regards to (1) CPU operations AND (2) GPU operations 
    //that are potentially occuring on other streams. Since all the previous kernel/memcpy operations use the same 
    //stream, they all occur sequentially with respect to our stream (which is necessary for correct execution)
  
    //Synchronize stream here since we want to do computations on the CPU
    HANDLE_ERROR( cudaStreamSynchronize(stream_) );

    //Compute the baseline (minimum) sampled cost
    float baseline = traj_costs_[0];
    for (int i = 0; i < NUM_ROLLOUTS; i++) {
      if (traj_costs_[i] < baseline){
        baseline = traj_costs_[i];
      }
    }

    //Now resume GPU computations
    launchNormExpKernel<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>(traj_costs_d_, gamma_, baseline, stream_);
    HANDLE_ERROR(cudaMemcpyAsync(traj_costs_.data(), traj_costs_d_, NUM_ROLLOUTS*sizeof(float), cudaMemcpyDeviceToHost, stream_));
    cudaStreamSynchronize(stream_);

    //Compute the normalizing term
    normalizer_ = 0;
    for (int i = 0; i < NUM_ROLLOUTS; i++) {
      normalizer_ += traj_costs_[i];
    }

    //Compute the cost weighted avergage.
    launchWeightedReductionKernel<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>(traj_costs_d_, du_d_, nu_d_, normalizer_, numTimesteps_, stream_);

    //Transfer control update to host.
    HANDLE_ERROR( cudaMemcpyAsync(du_.data(), du_d_, numTimesteps_*CONTROL_DIM*sizeof(float), cudaMemcpyDeviceToHost, stream_));
    cudaStreamSynchronize(stream_);

    //Save the control update
    for (int i = 0; i < numTimesteps_; i++) {
      for (int j = 0; j < CONTROL_DIM; j++) {
        U_[i*CONTROL_DIM + j] = du_[i*CONTROL_DIM + j];
      }
    }

  }
  //Smooth for the next optimization round
  savitskyGolay();
  //Compute the planned trajectory
  computeNominalTraj(state);
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
std::vector<float> MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::getControlSeq()
{
  return control_solution_;
}

template<class DYNAMICS_T, class COSTS_T, int ROLLOUTS, int BDIM_X, int BDIM_Y>
std::vector<float> MPPIController<DYNAMICS_T, COSTS_T, ROLLOUTS, BDIM_X, BDIM_Y>::getStateSeq()
{
  return state_solution_;
}
