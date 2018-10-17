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
 * @file generalized_linear.cuh
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Class definition for generalized linear models
 ***********************************************/

#ifndef GENERALIZED_LINEAR_CUH_
#define GENERALIZED_LINEAR_CUH_

#include "managed.cuh"
#include "meta_math.h"
#include "gpu_err_chk.h"
#include "cnpy.h"

#include <Eigen/Dense>

namespace autorally_control {

template<class BF, int S_DIM, int C_DIM, int BF_DIM, class K_FUNC, int K_DIM>
class GeneralizedLinear: public Managed
{
public:

  float2* control_rngs_;
  float2* control_rngs_d_;

  static const int NUM_BFS = BF_DIM;
  static const int STATE_DIM = S_DIM;
  static const int CONTROL_DIM = C_DIM;
  static const int DYNAMICS_DIM = STATE_DIM - K_DIM;
  static const int SHARED_MEM_REQUEST_GRD = DYNAMICS_DIM*BF_DIM;
  static const int SHARED_MEM_REQUEST_BLK = 0; 

  Eigen::Matrix<float, STATE_DIM, 1> state_der_; ///< The state derivative.

  GeneralizedLinear(Eigen::Matrix<float, DYNAMICS_DIM, NUM_BFS, Eigen::RowMajor> theta, float delta_t,
                    float2* control_rngs = NULL);

  GeneralizedLinear(float delta_t, float2* control_rngs = NULL);

  void setParams(Eigen::Matrix<float, DYNAMICS_DIM, NUM_BFS, Eigen::RowMajor> theta);

  void loadParams(std::string model_path);

  void paramsToDevice();

  void freeCudaMem();

  void enforceConstraints(Eigen::MatrixXf &state, Eigen::MatrixXf &control);

  void updateState(Eigen::MatrixXf &state, Eigen::MatrixXf &control);

  void computeKinematics(Eigen::MatrixXf &state);

  void computeDynamics(Eigen::MatrixXf &state, Eigen::MatrixXf &control);

  void updateModel(std::vector<int> description, std::vector<float> data){};

  __device__ void cudaInit(float* theta_s);

  __device__ void enforceConstraints(float* state, float* control);

  __device__ void computeKinematics(float* state, float* state_der);

  __device__ void computeStateDeriv(float* state, float* control, float* state_der, float* theta_s);

  __device__ void incrementState(float* state, float* state_der);

  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta_s);

protected:
  float dt_;
  BF* basisFunctions_;
  K_FUNC* kinematics_;

  //Host fields
  Eigen::Matrix<float, DYNAMICS_DIM, NUM_BFS, Eigen::RowMajor> theta_; ///< Coefficient matrix for basis function updates.
  Eigen::Matrix<float, NUM_BFS, 1> bf_vec_; ///< Vector of basis functions.

  //Device fields
  float* theta_d_; ///<Coefficient matrix in device memory.

};

}

#include "generalized_linear.cu"

#endif /*GENERALIZED_LINEAR_CUH_*/