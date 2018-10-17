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
* @file neural_net_model.cu
* @author Grady Williams <gradyrw@gmail.com>
* @date June 30, 2017
* @copyright 2016 Georgia Institute of Technology
* @brief NeuralNetModel class implementation.
*
***********************************************/

#define MPPI_NNET_NONLINEARITY(ans) tanh(ans)
#define MPPI_NNET_NONLINEARITY_DERIV(ans) (1 - powf(tanh(ans), 2))

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::NeuralNetModel(float delta_t, float2* control_rngs)
{
  dt_ = delta_t;
  if (control_rngs == NULL){
    control_rngs_ = new float2[CONTROL_DIM];
    for (int i = 0; i < CONTROL_DIM; i++){
      control_rngs_[i].x = -FLT_MAX;
      control_rngs_[i].y = FLT_MAX;
    }
  }
  else {
    control_rngs_ = control_rngs;
  }
  HANDLE_ERROR( cudaMalloc((void**)&control_rngs_d_, CONTROL_DIM*sizeof(float2)) );

  //Initialize GPU memory for constraints and model parameters.
  HANDLE_ERROR( cudaMalloc((void**)&theta_d_, NUM_PARAMS*sizeof(float)) );
  HANDLE_ERROR( cudaMalloc((void**)&stride_idcs_d_, (2*NUM_LAYERS + 1)*sizeof(int)) );
  HANDLE_ERROR( cudaMalloc((void**)&net_structure_d_, NUM_LAYERS*sizeof(int)) );
  //Initialize the host matrices of weights and biases
  weights_ = new Eigen::Matrix<float, -1, -1, Eigen::RowMajor>[NUM_LAYERS-1];
  biases_ = new Eigen::Matrix<float, -1, -1, Eigen::RowMajor>[NUM_LAYERS-1];
  net_params_ = new float[NUM_PARAMS];

  weighted_in_ = new Eigen::MatrixXf[NUM_LAYERS - 1];
  for (int i = 1; i < NUM_LAYERS; i++){
    weighted_in_[i-1] = Eigen::MatrixXf::Zero(net_structure_[i], 1);
  }
}

template<int S_DIM, int C_DIM,  int K_DIM, int... layer_args>
NeuralNetModel<S_DIM, C_DIM,  K_DIM, layer_args...>::~NeuralNetModel()
{}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::loadParams(std::string model_path)
{
  int i,j,k;
  std::string bias_name = "";
  std::string weight_name = "";
  if (!fileExists(model_path)){
    ROS_FATAL("Could not load neural net model at path: %s", model_path.c_str());
  }
  cnpy::npz_t param_dict = cnpy::npz_load(model_path);
  for (i = 1; i < NUM_LAYERS; i++){
    bias_name = "dynamics_b" + std::to_string(i);
    weight_name = "dynamics_W" + std::to_string(i);
    cnpy::NpyArray weight_i_raw = param_dict[weight_name];
    cnpy::NpyArray bias_i_raw = param_dict[bias_name];
    double* weight_i = weight_i_raw.data<double>();
    double* bias_i = bias_i_raw.data<double>();
    //Copy the data into eigen arrays.
    Eigen::MatrixXf weight_i_mat(net_structure_[i], net_structure_[i-1]);
    Eigen::MatrixXf bias_i_vec(net_structure_[i], 1);
    for (j = 0; j < net_structure_[i]; j++){
      for (k = 0; k < net_structure_[i-1]; k++){
        weight_i_mat(j,k) = (float)weight_i[j*net_structure_[i-1] + k];
      }
    }
    for (j = 0; j < net_structure_[i]; j++){
      bias_i_vec(j,0) = (float)bias_i[j];
    }
    weights_[i-1] = weight_i_mat;
    biases_[i-1] = bias_i_vec;
  }
  //Save parameters to GPU memory
  paramsToDevice();
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::setParams(Eigen::Matrix<float, -1, -1, Eigen::RowMajor>* weights, 
                                              Eigen::Matrix<float, -1, -1, Eigen::RowMajor>* biases)
{
  int i;
  for (i = 0; i < NUM_LAYERS - 1; i++){
    weights_[i] = weights[i];
    biases_[i] = biases[i];
  }
  paramsToDevice();
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::paramsToDevice()
{
  int i,j,k;
  int stride = 0;
  for (i = 0; i < NUM_LAYERS - 1; i++){
    //stride index for the weight matrix
    stride_idcs_[2*i] = stride;
    for (j = 0; j < net_structure_[i+1]; j++){
      for (k = 0; k < net_structure_[i]; k++){
        net_params_[stride + j*net_structure_[i] + k] = weights_[i](j,k);
      }
    }
    stride += net_structure_[i+1]*net_structure_[i];
    //stride index for the bias
    stride_idcs_[2*i + 1] = stride;
    for (j = 0; j < net_structure_[i+1]; j++){
      net_params_[stride + j] = biases_[i](j,0);
    }
    stride += net_structure_[i+1];
  }
  stride_idcs_[NUM_LAYERS*2] = stride;
  HANDLE_ERROR( cudaMemcpy(theta_d_, net_params_, NUM_PARAMS*sizeof(float), cudaMemcpyHostToDevice) );
  HANDLE_ERROR( cudaMemcpy(stride_idcs_d_, &stride_idcs_, (2*NUM_LAYERS + 1)*sizeof(int), cudaMemcpyHostToDevice) );
  HANDLE_ERROR( cudaMemcpy(net_structure_d_, &net_structure_, NUM_LAYERS*sizeof(int), cudaMemcpyHostToDevice) );
//If we're using constant memory transfer theta_d_ to it.
#if defined(MPPI_NNET_USING_CONSTANT_MEM___)
  HANDLE_ERROR( cudaMemcpyToSymbol(NNET_PARAMS, theta_d_, NUM_PARAMS*sizeof(float)) );
#endif /*MPPI_NNET_USING_CONSTANT_MEM___*/
  HANDLE_ERROR( cudaMemcpy(control_rngs_d_, control_rngs_, CONTROL_DIM*sizeof(float2), cudaMemcpyHostToDevice) );
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::updateModel(std::vector<int> description, std::vector<float> data)
{
  //First make sure that the network structure matches the current network structure
  bool validUpdate = true;
  for (int i = 0; i < description.size(); i++){
    if (description[i] != net_structure_[i]){
      validUpdate = false;
    }
  }
  //If the network structure is valid save the parameters
  if (validUpdate){
    int stride = 0;
    for (int i = 0; i < NUM_LAYERS - 1; i++){
      for (int j = 0; j < net_structure_[i+1]; j++){
        for (int k = 0; k < net_structure_[i]; k++){
          weights_[i](j,k) = data[stride + j*net_structure_[i] + k];
        }
      }
      stride += net_structure_[i+1]*net_structure_[i];
    }
    for (int i = 0; i < NUM_LAYERS - 1; i++){
      for (int j = 0; j < net_structure_[i+1]; j++){
        biases_[i](j,0) = data[stride + j];
      }
      stride += net_structure_[i+1];
    }
  }
}

template<int S_DIM, int C_DIM,  int K_DIM, int... layer_args>
void NeuralNetModel<S_DIM, C_DIM,  K_DIM, layer_args...>::printParamVec()
{
  int i;
  for (i = 0; i < NUM_PARAMS; i++){
    printf("Buffer Idx: %d, Value: %f \n", i, net_params_[i]);
  }
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::computeKinematics(Eigen::MatrixXf &state)
{
  state_der_(0) = cosf(state(2))*state(4) - sinf(state(2))*state(5);
  state_der_(1) = sinf(state(2))*state(4) + cosf(state(2))*state(5);
  state_der_(2) = -state(6); //Pose estimate actually gives the negative yaw derivative
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::computeDynamics(Eigen::MatrixXf &state, Eigen::MatrixXf &control)
{
  int i,j;
  Eigen::MatrixXf acts(net_structure_[0], 1);
  for (i = 0; i < DYNAMICS_DIM; i++){
    acts(i) = state(i + (STATE_DIM - DYNAMICS_DIM));
  }
  for (i = 0; i < CONTROL_DIM; i++){
    acts(DYNAMICS_DIM + i) = control(i);
  }
  for (i = 0; i < NUM_LAYERS - 1; i++){
    weighted_in_[i] = (weights_[i]*acts + biases_[i]).eval();
    acts = Eigen::MatrixXf::Zero(net_structure_[i+1], 1);
    if (i < NUM_LAYERS - 2) { //Last layer doesn't apply any non-linearity
      for (j = 0; j < net_structure_[i+1]; j++){
        acts(j) = MPPI_NNET_NONLINEARITY( (weighted_in_[i])(j) ); //Nonlinear component.
      }
    }
    else {
      for (j = 0; j < net_structure_[i+1]; j++){
        acts(j) = (weighted_in_[i])(j) ;
      }
    }
  }
  for (i = 0; i < DYNAMICS_DIM; i++){
    state_der_(i + (STATE_DIM - DYNAMICS_DIM)) = acts(i);
  }
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::computeGrad(Eigen::MatrixXf &state, Eigen::MatrixXf &control)
{
  jac_ = Eigen::Matrix<float, STATE_DIM, STATE_DIM + CONTROL_DIM>::Zero();

  //Start with the kinematic and physics model derivatives
  jac_.row(0) << 0, 0, -sin(state(2))*state(4) - cos(state(2))*state(5), 0, cos(state(2)), -sin(state(2)), 0, 0, 0;
  jac_.row(1) << 0, 0, cos(state(2))*state(4) - sin(state(2))*state(5), 0, sin(state(2)), cos(state(2)), 0, 0, 0;
  jac_.row(2) << 0, 0, 0, 0, 0, 0, -1, 0, 0;

  //First do the forward pass
  computeDynamics(state, control);

  //Start backprop
  ip_delta_ = Eigen::MatrixXf::Identity(DYNAMICS_DIM, DYNAMICS_DIM);
  Eigen::MatrixXf temp_delta = Eigen::MatrixXf::Identity(DYNAMICS_DIM, DYNAMICS_DIM);

  //Main backprop loop
  for (int i = NUM_LAYERS-2; i > 0; i--){
    Eigen::MatrixXf zp = weighted_in_[i-1];
    for (int j = 0; j < net_structure_[i]; j++){
      zp(j) = MPPI_NNET_NONLINEARITY_DERIV(zp(j));
    }
    ip_delta_ =  ( (weights_[i]).transpose()*ip_delta_ ).eval();
    for (int j = 0; j < DYNAMICS_DIM; j++){
      ip_delta_.col(j) = ip_delta_.col(j).array() * zp.array();
    }
  }
  //Finish the backprop loop
  ip_delta_ = ( ((weights_[0]).transpose())*ip_delta_).eval();
  jac_.bottomRightCorner(DYNAMICS_DIM, DYNAMICS_DIM + CONTROL_DIM) += ip_delta_.transpose();
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::enforceConstraints(Eigen::MatrixXf &state, Eigen::MatrixXf &control)
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

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::updateState(Eigen::MatrixXf &state, Eigen::MatrixXf &control)
{
  enforceConstraints(state, control);
  computeKinematics(state);
  computeDynamics(state, control);
  state += state_der_*dt_;
  state_der_ *= 0;
}

template<int S_DIM, int C_DIM,  int K_DIM, int... layer_args>
void NeuralNetModel<S_DIM, C_DIM,  K_DIM, layer_args...>::freeCudaMem()
{
  HANDLE_ERROR(cudaFree(theta_d_));
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
__device__ void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::cudaInit(float* theta_s)
{}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
__device__ void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::printCudaParamVec()
{
  int i;
  if (threadIdx.x == 0 && blockIdx.x == 0) {
    for (i = 0; i < NUM_PARAMS; i++){
      printf("Buffer Idx: %d, Value: %f \n", i, theta_d_[i]);
    }
  }
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
__device__ void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::enforceConstraints(float* state, float* control)
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

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
__device__ void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::computeStateDeriv(float* state, float* control, float* state_der, float* theta_s)
{
  if (threadIdx.y == 0){
    computeKinematics(state, state_der);
  }
  computeDynamics(state, control, state_der, theta_s);
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
__device__ void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::incrementState(float* state, float* state_der)
{
  int i;
  int tdy = threadIdx.y;
  //Add the state derivative time dt to the current state.
  for (i = tdy; i < STATE_DIM; i+=blockDim.y){
    state[i] += state_der[i]*dt_;
    state_der[i] = 0; //Important: reset the state derivative to zero.
  }
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
__device__ void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::computeKinematics(float* state, float* state_der)
{
  state_der[0] = cosf(state[2])*state[4] - sinf(state[2])*state[5];
  state_der[1] = sinf(state[2])*state[4] + cosf(state[2])*state[5];
  state_der[2] = -state[6]; //Pose estimate actually gives the negative yaw derivative
}

template<int S_DIM, int C_DIM, int K_DIM, int... layer_args>
__device__ void NeuralNetModel<S_DIM, C_DIM, K_DIM, layer_args...>::computeDynamics(float* state, float* control, float* state_der, float* theta_s)
{
  float* curr_act;
  float* next_act;
  float* tmp_act;
  float tmp;
  float* W;
  float* b;
  int tdx = threadIdx.x;
  int tdy = threadIdx.y;
  int tdz = threadIdx.z;
  int i,j,k;
  curr_act = &theta_s[(2*LARGEST_LAYER)*(blockDim.x*tdz + tdx)];
  next_act = &theta_s[(2*LARGEST_LAYER)*(blockDim.x*tdz + tdx) + LARGEST_LAYER];
  for (i = tdy; i < DYNAMICS_DIM; i+= blockDim.y){
    curr_act[i] = state[i + (STATE_DIM - DYNAMICS_DIM)];
  }
  for (i = tdy; i < CONTROL_DIM; i+= blockDim.y){
    curr_act[DYNAMICS_DIM + i] = control[i];
  }
  __syncthreads();
  for (i = 0; i < NUM_LAYERS - 1; i++){
  //Conditional compilation depending on if we're using a global constant memory array or not.
  #if defined(MPPI_NNET_USING_CONSTANT_MEM___) //Use constant memory.
    W = &NNET_PARAMS[stride_idcs_d_[2*i]];
    b = &NNET_PARAMS[stride_idcs_d_[2*i + 1]];
  #else //Use (slow) global memory.
    W = &theta_d_[stride_idcs_d_[2*i]];
    b = &theta_d_[stride_idcs_d_[2*i + 1]];
  #endif
    for (j = tdy; j < net_structure_d_[i+1]; j += blockDim.y) {
      tmp = 0;
      for (k = 0; k < net_structure_d_[i]; k++){
        //No atomic add necessary.
        tmp += W[j*net_structure_d_[i] + k]*curr_act[k];
      }
      tmp += b[j];
      if (i < NUM_LAYERS - 2){
        tmp = MPPI_NNET_NONLINEARITY(tmp);
      }
      next_act[j] = tmp;
    }
    //Swap the two pointers
    tmp_act = curr_act;
    curr_act = next_act;
    next_act = tmp_act;
    __syncthreads();
  }
  for (i = tdy; i < DYNAMICS_DIM; i+= blockDim.y){
    state_der[i + (STATE_DIM - DYNAMICS_DIM)] = curr_act[i];
  }
  __syncthreads();
}
