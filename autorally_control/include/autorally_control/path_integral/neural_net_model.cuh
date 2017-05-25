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

#include <eigen3/Eigen/Dense>

#include "meta_math.h"
#include "gpu_err_chk.h"
#include "cnpy.h"

namespace autorally_control {

template<int S_DIM, int C_DIM, class K_FUNC, int K_DIM, int... layer_args>
class NeuralNetModel: public Managed
{

public:

  static const int STATE_DIM = S_DIM;
  static const int CONTROL_DIM = C_DIM;
  static const int DYNAMICS_DIM = STATE_DIM - K_DIM;
  static const int NUM_LAYERS = layer_counter(layer_args...); ///< Total number of layers (including in/out layer)
  static const int LARGEST_LAYER = neuron_counter(layer_args...); ///< Number of neurons in the largest layer(including in/out neurons)
  static const int NUM_PARAMS = param_counter(layer_args...); ///< Total number of model parameters;
  static const int SHARED_MEM_REQUEST_GRD = 0; ///< Amount of shared memory we need per BLOCK.
  static const int SHARED_MEM_REQUEST_BLK = 2*neuron_counter(layer_args...); ///< Amount of shared memory we need per ROLLOUT.

  float* theta_d_; ///< GPU memory parameter array.
  int* stride_idcs_d_; ///< GPU memory for keeping track of parameter strides
  int* net_structure_d_; ///GPU memory for keeping track of the neural net structure.
  float2* control_rngs_;

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

  void computeDynamics(Eigen::MatrixXf &state, Eigen::MatrixXf &control);


  __device__ void cudaInit(float* theta_s);

  __device__ void enforceConstraints(float* state, float* control);

  __device__ void computeStateDeriv(float* state, float* control, float* state_der, float* theta_s);

  __device__ void incrementState(float* state, float* state_der);

  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta_s);

  __device__ void printCudaParamVec();

private:
  float dt_;
  K_FUNC* kinematics_;

  //Neural net structure
  int net_structure_[NUM_LAYERS] = {layer_args...};
  int stride_idcs_[NUM_LAYERS*2 + 1] = {0};

  //Host fields
  Eigen::Matrix<float, -1, -1, Eigen::RowMajor>* weights_; ///< Matrices of weights {W_1, W_2, ... W_n}
  Eigen::Matrix<float, -1, -1, Eigen::RowMajor>* biases_; ///< Vectors of biases {b_1, b_2, ... b_n}
  float* net_params_; 

  Eigen::Matrix<float, STATE_DIM, 1> state_der_; ///< The state derivative.
  float2* control_rngs_d_;
};

#include "neural_net_model.cut"

}

#endif /* NEURAL_NET_MODEL_CUH_ */
