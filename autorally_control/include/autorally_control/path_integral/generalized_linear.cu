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
 * @file generalized_linear.cu
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Class implementation for generalized linear models
 ***********************************************/

namespace autorally_control {

#include "gpu_err_chk.h"

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::GeneralizedLinear(
                    Eigen::Matrix<float, DYNAMICS_DIM, NUM_BFS, Eigen::RowMajor> theta, 
                    float delta_t, float2* control_rngs)
{
  int i;
  dt_ = delta_t;
  basisFunctions_ = new BF();
  kinematics_ = new K_FUNC();
  if (control_rngs == NULL){
    control_rngs_ = new float2[CONTROL_DIM];
    for (i = 0; i < CONTROL_DIM; i++){
      control_rngs_[i].x = -FLT_MAX;
      control_rngs_[i].y = FLT_MAX;
    }
  }
  else {
    control_rngs_ = control_rngs;
  }
  HANDLE_ERROR( cudaMalloc((void**)&control_rngs_d_, CONTROL_DIM*sizeof(float2)) );
  HANDLE_ERROR( cudaMalloc((void**)&theta_d_, NUM_BFS*DYNAMICS_DIM*sizeof(float)) );
  setParams(theta);
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::GeneralizedLinear(float delta_t, float2* control_rngs)
{
  int i;
  dt_ = delta_t;
  basisFunctions_ = new BF();
  kinematics_ = new K_FUNC();
  if (control_rngs == NULL){
    control_rngs_ = new float2[CONTROL_DIM];
    for (i = 0; i < CONTROL_DIM; i++){
      control_rngs_[i].x = -FLT_MAX;
      control_rngs_[i].y = FLT_MAX;
    }
  }
  else {
    control_rngs_ = control_rngs;
  }
  HANDLE_ERROR( cudaMalloc((void**)&control_rngs_d_, CONTROL_DIM*sizeof(float2)) );
  HANDLE_ERROR( cudaMalloc((void**)&theta_d_, NUM_BFS*DYNAMICS_DIM*sizeof(float)) );
  HANDLE_ERROR( cudaMemcpy(control_rngs_d_, control_rngs_, CONTROL_DIM*sizeof(float2), cudaMemcpyHostToDevice) );
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::setParams(Eigen::Matrix<float, DYNAMICS_DIM, NUM_BFS, Eigen::RowMajor> theta)
{
  theta_ = theta;
  paramsToDevice(); //Save to GPU memory
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::loadParams(std::string model_path)
{
  int i,j;
  if (!fileExists(model_path)){
    ROS_FATAL("Could not load generalized linear model at path: %s", model_path.c_str());
  }
  Eigen::Matrix<float, DYNAMICS_DIM, NUM_BFS, Eigen::RowMajor> theta;
  cnpy::npz_t param_dict = cnpy::npz_load(model_path);
  cnpy::NpyArray theta_raw = param_dict["W"];
  double* theta_ptr = theta_raw.data<double>();
  for (i = 0; i < DYNAMICS_DIM; i++){
    for (j = 0; j < NUM_BFS; j++){
      theta(i,j) = (float)theta_ptr[i*NUM_BFS + j];
    }
  }
  setParams(theta);
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::paramsToDevice()
{
  //Transfer CPU params to the GPU
  HANDLE_ERROR( cudaMemcpy(theta_d_, theta_.data(), NUM_BFS*DYNAMICS_DIM*sizeof(float), cudaMemcpyHostToDevice) );
  HANDLE_ERROR( cudaMemcpy(control_rngs_d_, control_rngs_, CONTROL_DIM*sizeof(float2), cudaMemcpyHostToDevice) );
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::freeCudaMem()
{
  HANDLE_ERROR(cudaFree(theta_d_));
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::enforceConstraints(Eigen::MatrixXf &state, Eigen::MatrixXf &control)
{
  int i;
  for (i = 0; i < CONTROL_DIM; i++){
    if (control(i) < control_rngs_[i].x){
      control(i) = control_rngs_[i].x;
    }
    else if (control(i) > control_rngs_[i].y){
      control(i) = control_rngs_[i].y;
    }
  }
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::updateState(Eigen::MatrixXf &state, Eigen::MatrixXf &control)
{
  enforceConstraints(state, control);
  computeKinematics(state);
  computeDynamics(state, control);
  state += state_der_*dt_;
  state_der_ *= 0;
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::computeKinematics(Eigen::MatrixXf &state)
{
  state_der_(0) = cosf(state(2))*state(4) - sinf(state(2))*state(5);
  state_der_(1) = sinf(state(2))*state(4) + cosf(state(2))*state(5);
  state_der_(2) = -state(6); //Pose estimate actually gives the negative yaw derivative
}


template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::computeDynamics(Eigen::MatrixXf &state, Eigen::MatrixXf &control)
{
  int i;
  //Now compute the basis functions.
  for (i = 0; i < NUM_BFS; i++){
    bf_vec_(i) = basisFunctions_->basisFuncX(i, state.data(), control.data());
  }
  state_der_.block(STATE_DIM - DYNAMICS_DIM, 0, DYNAMICS_DIM, 1) = theta_*bf_vec_;
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
__device__ void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::cudaInit(float* theta_s)
{
  //Transfers the global memory cuda data to a shared memory array.
  int i,j;
  int tdx = threadIdx.x;
  int tdy = threadIdx.y;
  for (i = tdy; i < DYNAMICS_DIM; i+= blockDim.y) {
    for (j = tdx; j < NUM_BFS; j+= blockDim.x) {
      theta_s[i*NUM_BFS + j] = theta_d_[i*NUM_BFS + j];
    }
  }
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
__device__ void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::enforceConstraints(float* state, float* control)
{
  int i;
  for (i = 0; i < CONTROL_DIM; i++){
    if (control[i] < control_rngs_d_[i].x){
      control[i] = control_rngs_d_[i].x;
    }
    else if (control[i] > control_rngs_d_[i].y){
      control[i] = control_rngs_d_[i].y;
    }
  }
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
__device__ void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::computeStateDeriv(float* s, float* u, float* s_der, float* theta_s)
{
  if (threadIdx.y == 0){
    computeKinematics(s, s_der);
  }
  computeDynamics(s, u, s_der, theta_s);
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
__device__ void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::incrementState(float* state, float* state_der)
{
  int i;
  int tdy = threadIdx.y;
  //Add the state derivative time dt to the current state.
  for (i = tdy; i < STATE_DIM; i+=blockDim.y){
    state[i] += state_der[i]*dt_;
    state_der[i] = 0; //Important: reset the state derivative to zero.
  }
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
__device__ void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::computeKinematics(float* state, float* state_der)
{
  state_der[0] = cosf(state[2])*state[4] - sinf(state[2])*state[5];
  state_der[1] = sinf(state[2])*state[4] + cosf(state[2])*state[5];
  state_der[2] = -state[6]; //Pose estimate actually gives the negative yaw derivative
}

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
__device__ void GeneralizedLinear<BF, S_DIM, C_DIM, BF_DIM, K_FUNC, K_DIM>::computeDynamics(float* s, float* u, float* s_der, float* theta_s)
{
  int i,j;
  int tdy = threadIdx.y;
  float bf_temp; //Temporary variable for storing basis function evaluations.
  float eval_temp[DYNAMICS_DIM]; //Temporary variable to reduce the number of atomic adds.
  for (i = 0; i < DYNAMICS_DIM; i++){
    eval_temp[i] = 0;
  }
  for (i = tdy; i < NUM_BFS; i += blockDim.y) {
    bf_temp = basisFunctions_->basisFuncX(i, s, u);
    for (j = 0; j < DYNAMICS_DIM; j++) {
      eval_temp[j] += theta_s[j*NUM_BFS + i]*bf_temp;
    }
  }
  //Add to state using atomic add.
  for (i = 0; i < DYNAMICS_DIM; i++){
    atomicAdd(&s_der[i+(STATE_DIM - DYNAMICS_DIM)], eval_temp[i]);
  }
}

}
