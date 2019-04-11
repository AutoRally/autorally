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
* @file neural_net_model.cuh
* @author Grady Williams <gradyrw@gmail.com>
* @date May 24, 2017
* @copyright 2017 Georgia Institute of Technology
* @brief Neural Network Model class definition
*
***********************************************/

#ifndef NEURAL_NET_MODEL_CUH_
#define NEURAL_NET_MODEL_CUH_

#include "managed.cuh"
#include "meta_math.h"
#include "param_getter.h"
#include "gpu_err_chk.h"
#include "cnpy.h"

#include <Eigen/Dense>

namespace autorally_control {

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
class NeuralNetModel: public Managed
{

public:

  static const int STATE_DIM = S_DIM;
  static const int CONTROL_DIM = C_DIM;
  static const int DYNAMICS_DIM = STATE_DIM - K_DIM;
  static const int NUM_LAYERS = layer_counter(layer_args...); ///< Total number of layers (including in/out layer)
  static const int PRIME_PADDING = 1; ///< Extra padding to largest layer to avoid shared mem bank conflicts  
  static const int LARGEST_LAYER = neuron_counter(layer_args...) + PRIME_PADDING; ///< Number of neurons in the largest layer(including in/out neurons)
  static const int NUM_PARAMS = param_counter(layer_args...); ///< Total number of model parameters;
  static const int SHARED_MEM_REQUEST_GRD = 0; ///< Amount of shared memory we need per BLOCK.
  static const int SHARED_MEM_REQUEST_BLK = 2*LARGEST_LAYER; ///< Amount of shared memory we need per ROLLOUT.

  float* theta_d_; ///< GPU memory parameter array.
  int* stride_idcs_d_; ///< GPU memory for keeping track of parameter strides
  int* net_structure_d_; ///GPU memory for keeping track of the neural net structure.
  float2* control_rngs_;

  Eigen::Matrix<float, STATE_DIM, 1> state_der_; ///< The state derivative.
  float2* control_rngs_d_;

  Eigen::MatrixXf ip_delta_; ///< The neural net state derivative.
  Eigen::Matrix<float, STATE_DIM, STATE_DIM + CONTROL_DIM> jac_; //Total state derivative

  NeuralNetModel(float delta_t, float2* control_rngs = NULL);

  ~NeuralNetModel();

  void loadParams(std::string model_path);

  void setParams(Eigen::Matrix<float, -1, -1, Eigen::RowMajor>* weights, 
                 Eigen::Matrix<float, -1, -1, Eigen::RowMajor>* biases);

  void paramsToDevice();

  void printParamVec();

  void freeCudaMem();

  void enforceConstraints(Eigen::MatrixXf &state, Eigen::MatrixXf &control);

  void updateState(Eigen::MatrixXf &state, Eigen::MatrixXf &control);

  void computeKinematics(Eigen::MatrixXf &state);

  void computeDynamics(Eigen::MatrixXf &state, Eigen::MatrixXf &control);

  void computeGrad(Eigen::MatrixXf &state, Eigen::MatrixXf &control);

  void updateModel(std::vector<int> description, std::vector<float> data);

  __device__ void computeKinematics(float* state, float* state_der);

  __device__ void cudaInit(float* theta_s);

  __device__ void enforceConstraints(float* state, float* control);

  __device__ void computeStateDeriv(float* state, float* control, float* state_der, float* theta_s);

  __device__ void incrementState(float* state, float* state_der);

  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta_s);

  __device__ void printCudaParamVec();

private:
  float dt_;

  //Neural net structure
  int net_structure_[NUM_LAYERS] = {layer_args...};
  int stride_idcs_[NUM_LAYERS*2 + 1] = {0};

  //Host fields
  Eigen::Matrix<float, -1, -1, Eigen::RowMajor>* weights_; ///< Matrices of weights {W_1, W_2, ... W_n}
  Eigen::Matrix<float, -1, -1, Eigen::RowMajor>* biases_; ///< Vectors of biases {b_1, b_2, ... b_n}

  Eigen::MatrixXf* weighted_in_;

  float* net_params_; 
};

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
const int NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::STATE_DIM;

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
const int NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::CONTROL_DIM;

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
const int NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::DYNAMICS_DIM;

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
const int NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::NUM_LAYERS; 

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
const int NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::LARGEST_LAYER;

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
const int NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::NUM_PARAMS;

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
const int NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::SHARED_MEM_REQUEST_GRD;

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
const int NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::SHARED_MEM_REQUEST_BLK;

#include "neural_net_model.cu"

}

#endif /* NEURAL_NET_MODEL_CUH_ */
