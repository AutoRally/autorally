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
 * @file costs.cu
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief MPPICosts class implementation
 ***********************************************/
#include <stdio.h>
#include <stdlib.h>

#include "gpu_err_chk.h"
#include "debug_kernels.cuh"

namespace autorally_control {

inline MPPICosts::MPPICosts(int width, int height)
{
  width_ = width;
  height_ = height;
  allocateTexMem();
  //Initialize memory for device cost param struct
  HANDLE_ERROR( cudaMalloc((void**)&params_d_, sizeof(CostParams)) );
  debugging_ = false;

  callback_f_ = boost::bind(&MPPICosts::updateParams_dcfg, this, _1, _2);
  server_.setCallback(callback_f_);
}

inline MPPICosts::MPPICosts(ros::NodeHandle mppi_node)
{
  //Transform from world coordinates to normalized grid coordinates
  Eigen::Matrix3f R;
  Eigen::Array3f trs;
  HANDLE_ERROR( cudaMalloc((void**)&params_d_, sizeof(CostParams)) ); //Initialize memory for device cost param struct
  //Get the map path
  std::string map_path;
  mppi_node.getParam("map_path", map_path);
  std::vector<float> track_costs = loadTrackData(map_path.c_str(), R, trs); //R and trs passed by reference
  updateTransform(R, trs);
  updateParams(mppi_node);
  allocateTexMem();
  costmapToTexture(track_costs.data());
  debugging_ = false;

  callback_f_ = boost::bind(&MPPICosts::updateParams_dcfg, this, _1, _2);
  server_.setCallback(callback_f_);
}

inline MPPICosts::~MPPICosts()
{}

inline void MPPICosts::allocateTexMem()
{
  //Allocate memory for the cuda array which is bound the costmap_tex_
  channelDesc_ = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
  HANDLE_ERROR(cudaMallocArray(&costmapArray_d_, &channelDesc_, width_, height_));
}

inline void MPPICosts::costmapToTexture(float* costmap)
{
  //Transfer CPU mem to GPU
  HANDLE_ERROR(cudaMemcpyToArray(costmapArray_d_, 0, 0, costmap, width_*height_*sizeof(float), cudaMemcpyHostToDevice));

  //Specify texture
  struct cudaResourceDesc resDesc;
  memset(&resDesc, 0, sizeof(resDesc));
  resDesc.resType = cudaResourceTypeArray;
  resDesc.res.array.array = costmapArray_d_;

  //Specify texture object parameters
  struct cudaTextureDesc texDesc;
  memset(&texDesc, 0, sizeof(texDesc));
  texDesc.addressMode[0] = cudaAddressModeClamp;
  texDesc.addressMode[1] = cudaAddressModeClamp;
  texDesc.filterMode = cudaFilterModeLinear;
  texDesc.readMode = cudaReadModeElementType;
  texDesc.normalizedCoords = 1;

  //First destroy the current texture object
  HANDLE_ERROR(cudaDestroyTextureObject(costmap_tex_));

  //Now create the new texture object.
  HANDLE_ERROR(cudaCreateTextureObject(&costmap_tex_, &resDesc, &texDesc, NULL) );
}

inline void MPPICosts::update(Eigen::MatrixXf state)
{}

inline void MPPICosts::updateParams_dcfg(autorally_control::PathIntegralParamsConfig &config, int lvl)
{
  params_.desired_speed = (float)config.desired_speed;
  params_.speed_coeff = (float)config.speed_coefficient;
  params_.track_coeff = (float)config.track_coefficient;
  params_.max_slip_ang = (float)config.max_slip_angle;
  params_.slip_penalty = (float)config.slip_penalty;
  params_.crash_coeff = (float)config.crash_coefficient;
  params_.track_slop = (float)config.track_slop;
  params_.steering_coeff = (float)config.steering_coeff;
  params_.throttle_coeff = (float)config.throttle_coeff;
  paramsToDevice();
}

inline void MPPICosts::updateParams(ros::NodeHandle mppi_node)
{
  double desired_speed, speed_coeff, track_coeff, max_slip_ang, 
          slip_penalty, track_slop, crash_coeff, steering_coeff, throttle_coeff;
  int num_timesteps;
  //Read parameters from the ROS parameter server
  mppi_node.getParam("desired_speed", desired_speed);
  mppi_node.getParam("speed_coefficient", speed_coeff);
  mppi_node.getParam("track_coefficient", track_coeff);
  mppi_node.getParam("max_slip_angle", max_slip_ang);
  mppi_node.getParam("slip_penalty", slip_penalty);
  mppi_node.getParam("track_slop", track_slop);
  mppi_node.getParam("crash_coeff", crash_coeff);
  mppi_node.getParam("steering_coeff", steering_coeff);
  mppi_node.getParam("throttle_coeff", throttle_coeff);
  mppi_node.getParam("num_timesteps", num_timesteps);

  //Transfer to the cost params struct
  params_.desired_speed = (float)desired_speed;
  params_.speed_coeff = (float)speed_coeff;
  params_.track_coeff = (float)track_coeff;
  params_.max_slip_ang = (float)max_slip_ang;
  params_.slip_penalty = (float)slip_penalty;
  params_.track_slop = (float)track_slop;
  params_.crash_coeff = (float)crash_coeff;
  params_.steering_coeff = (float)steering_coeff;
  params_.throttle_coeff = (float)throttle_coeff;
  params_.num_timesteps = num_timesteps;
  //Move the updated parameters to gpu memory
  paramsToDevice();
}

inline void MPPICosts::updateTransform(Eigen::MatrixXf m, Eigen::ArrayXf trs){
  params_.r_c1.x = m(0,0);
  params_.r_c1.y = m(1,0);
  params_.r_c1.z = m(2,0);
  params_.r_c2.x = m(0,1);
  params_.r_c2.y = m(1,1);
  params_.r_c2.z = m(2,1);
  params_.trs.x = trs(0);
  params_.trs.y = trs(1);
  params_.trs.z = trs(2);
  //Move the updated parameters to gpu memory
  paramsToDevice();
}

inline std::vector<float> MPPICosts::loadTrackData(const char* costmap_path, Eigen::Matrix3f &R, Eigen::Array3f &trs)
{
  int i;
  float p;
  FILE *track_data_file;
  char file_path[256];
  file_path[0] = 0;
  strcat(file_path, costmap_path);
  strcat(file_path, "track_data.txt");
  track_data_file=fopen(file_path, "r");
  if (track_data_file == NULL) {
  	ROS_INFO("Error opening track data file: No such file or directory: %s \n", file_path);
  	ros::shutdown();
  }
  //Read the parameters from the file
  float x_min, x_max, y_min, y_max, resolution;
  bool success = true;
  success = success && fscanf(track_data_file, "%f", &x_min);
  success = success && fscanf(track_data_file, "%f", &x_max);
  success = success && fscanf(track_data_file, "%f", &y_min);
  success = success && fscanf(track_data_file, "%f", &y_max);
  success = success && fscanf(track_data_file, "%f", &resolution);
  //Save width_ and height_ parameters
  width_ = int((x_max - x_min)*resolution);
  height_ = int((y_max - y_min)*resolution);
  std::vector<float> track_costs(width_*height_);
  //Scan the result of the file to load track parameters
  for (i = 0; i < width_*height_; i++) {
  	success = success && fscanf(track_data_file, "%f", &p);
  	track_costs[i] = p;
  }
  if (!success){
    ROS_INFO("Warning track parameters not read succesfully.");
  }
  //Save the scaling and offset
  R << 1./(x_max - x_min), 0,                  0,
       0,                  1./(y_max - y_min), 0,
       0,                  0,                  1;
  trs << -x_min/(x_max - x_min), -y_min/(y_max - y_min), 1;
  fclose(track_data_file);
  return track_costs;
}

inline void MPPICosts::paramsToDevice()
{
  HANDLE_ERROR( cudaMemcpy(params_d_, &params_, sizeof(CostParams), cudaMemcpyHostToDevice) );
}

inline void MPPICosts::getCostInfo()
{
}

inline float MPPICosts::getDesiredSpeed()
{
  return params_.desired_speed;
}

inline void MPPICosts::setDesiredSpeed(float desired_speed)
{
  params_.desired_speed = desired_speed;
  paramsToDevice();
}

inline void MPPICosts::debugDisplayInit()
{
  debugDisplayInit(10, 10, 50);
}

inline void MPPICosts::debugDisplayInit(int width_m, int height_m, int ppm)
{
  debug_img_width_ = width_m;
  debug_img_height_ = height_m;
  debug_img_ppm_ = ppm;
  debug_img_size_ = (width_m*ppm)*(height_m*ppm);
  debug_data_ = new float[debug_img_size_];
  debugging_ = true;
  HANDLE_ERROR( cudaMalloc((void**)&debug_data_d_, debug_img_size_*sizeof(float)) );
}

inline void MPPICosts::debugDisplay(float x, float y)
{
  if (!debugging_){
    debugDisplayInit();
  }
  launchDebugCostKernel(x, y, debug_img_width_, debug_img_height_, debug_img_ppm_, 
                        costmap_tex_, debug_data_d_, params_.r_c1, params_.r_c2, params_.trs);
  //Now we just have to display debug_data_d_
  HANDLE_ERROR( cudaMemcpy(debug_data_, debug_data_d_, debug_img_size_*sizeof(float), cudaMemcpyDeviceToHost) );
  debug_img_ = cv::Mat(debug_img_width_*debug_img_ppm_, debug_img_height_*debug_img_ppm_, CV_32F, debug_data_);
  cv::namedWindow("debugImage", cv::WINDOW_AUTOSIZE);
  cv::imshow("debugImage", debug_img_);
  cv::waitKey(1);
}

inline void MPPICosts::freeCudaMem()
{
  HANDLE_ERROR(cudaDestroyTextureObject(costmap_tex_));
  HANDLE_ERROR(cudaFreeArray(costmapArray_d_));
  HANDLE_ERROR(cudaFree(params_d_));
  if (debugging_) {
    HANDLE_ERROR(cudaFree(debug_data_d_));
  }
}

inline __host__ __device__ void MPPICosts::getCrash(float* state, int* crash) {
  if (fabs(state[3]) > 1.57) {
    crash[0] = 1;
  }

}

inline __host__ __device__ float MPPICosts::getControlCost(float* u, float* du, float* vars)
{
  float control_cost = 0;
  control_cost += params_d_->steering_coeff*du[0]*(u[0] - du[0])/(vars[0]*vars[0]);
  control_cost += params_d_->throttle_coeff*du[1]*(u[1] - du[1])/(vars[1]*vars[1]);
  return control_cost;
}

inline __host__ __device__ float MPPICosts::getSpeedCost(float* s, int* crash)
{
  float cost = 0;
  if (params_d_->desired_speed < 999.0){
    float speed = 0;
    if (s[4] > 0){
      speed = s[4];
      cost = params_d_->speed_coeff*powf(speed - params_d_->desired_speed, 2);
    }else {
      cost = params_d_->speed_coeff*powf(speed + params_d_->desired_speed, 2);
    }
  }
  else{//Turn the speed cost into a reward (negative cost), never could get this to work
    if (crash[0] > 0){
      cost = 0.0;
    }
    else if (s[4] > 0.001 && fabs(-atan(s[5]/fabs(s[4]))) > params_d_->max_slip_ang){
      cost = 0.0;
    }
    else {
      cost = -params_d_->speed_coeff*s[4]*s[4];
    }
  }
  return cost;
}

inline __host__ __device__ float MPPICosts::getCrashCost(float* s, int* crash, int timestep)
{
  float crash_cost = 0;
  if (crash[0] > 0) {
      crash_cost = params_d_->crash_coeff;
  }
  return crash_cost;
}

inline __host__ __device__ float MPPICosts::getStabilizingCost(float* s)
{
  float stabilizing_cost = 0;
  if (fabs(s[4]) > 0.001) {
    float slip = -atan(s[5]/fabs(s[4]));
    stabilizing_cost = params_d_->slip_penalty*powf(slip,2);
    if (fabs(-atan(s[5]/fabs(s[4]))) > params_d_->max_slip_ang) {
      //If the slip angle is above the max slip angle kill the trajectory.
      stabilizing_cost += 100000.0;
    }
  }
  return stabilizing_cost;
}

inline __host__ __device__ void MPPICosts::coorTransform(float x, float y, float* u, float* v, float* w)
{
  //Compute a projective transform of (x, y, 0, 1)
  u[0] = params_d_->r_c1.x*x + params_d_->r_c2.x*y + params_d_->trs.x;
  v[0] = params_d_->r_c1.y*x + params_d_->r_c2.y*y + params_d_->trs.y;
  w[0] = params_d_->r_c1.z*x + params_d_->r_c2.z*y + params_d_->trs.z;
}

inline __device__ float MPPICosts::getTrackCost(float* s, int* crash)
{
  float track_cost = 0;

  //Compute a transformation to get the (x,y) positions of the front and back of the car.
  float x_front = s[0] + FRONT_D*__cosf(s[2]);
  float y_front = s[1] + FRONT_D*__sinf(s[2]);
  float x_back = s[0] + BACK_D*__cosf(s[2]);
  float y_back = s[1] + BACK_D*__sinf(s[2]);

  float u,v,w; //Transformed coordinates

  //Cost of front of the car
  coorTransform(x_front, y_front, &u, &v, &w);
  float track_cost_front = tex2D<float>(costmap_tex_, u/w, v/w); 

  //Cost for back of the car
  coorTransform(x_back, y_back, &u, &v, &w);
  float track_cost_back = tex2D<float>(costmap_tex_, u/w, v/w);

  track_cost = (fabs(track_cost_front) + fabs(track_cost_back) )/2.0;
  if (fabs(track_cost) < params_d_->track_slop) {
    track_cost = 0;
  }
  else {
    track_cost = params_d_->track_coeff*track_cost;
  }
  if (fabs(track_cost_front) >= .95 || fabs(track_cost_back) >= .95) {
    crash[0] = 1;
  }
  return track_cost;
}

//Compute the immediate running cost.
inline __device__ float MPPICosts::computeCost(float* s, float* u, float* du, 
                                        float* vars, int* crash, int timestep)
{
  float control_cost = getControlCost(u, du, vars);
  float track_cost = getTrackCost(s, crash);
  float speed_cost = getSpeedCost(s, crash);
  float crash_cost = powf(DISCOUNT, timestep)*getCrashCost(s, crash, timestep); //Time decaying crash penalty
  float stabilizing_cost = getStabilizingCost(s);            
  float cost = control_cost + speed_cost + crash_cost + track_cost + stabilizing_cost;
  if (cost > 1e9 || isnan(cost)) {
    cost = 1e9;
  }
  return cost;
}

inline __device__ float MPPICosts::terminalCost(float* s)
{
  return 0.0;
}

}

