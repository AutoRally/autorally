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
3* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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
 * @file costs_pc.cu
 * @author Jake Sacks <jsacks@gatech.edu>
 * @date December 11, 2018
 * @copyright 2018 Georgia Institute of Technology
 * @brief MPPIPCCosts class implementation
 ***********************************************/

namespace autorally_control {

inline MPPIPCCosts::MPPIPCCosts(ros::NodeHandle mppi_node) : MPPICosts(mppi_node)
{
  HANDLE_ERROR( cudaMalloc((void**)&obs_params_d_, sizeof(ObsParams)) );

  obstacle_costs_ = std::vector<float>(width_ * height_, 0.);
  obsChannelDesc_ = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
  HANDLE_ERROR(cudaMallocArray(&obstaclemapArray_d_, &obsChannelDesc_, width_, height_));

  float* obstaclemap_ptr = obstacle_costs_.data();
  HANDLE_ERROR( cudaMemcpyToArray(obstaclemapArray_d_, 0, 0, obstaclemap_ptr, width_*height_*sizeof(float),
                                  cudaMemcpyHostToDevice) );

  //Specify texture
  memset(&resDesc_, 0, sizeof(resDesc_));
  resDesc_.resType = cudaResourceTypeArray;
  resDesc_.res.array.array = obstaclemapArray_d_;

  //Specify texture object parameters
  memset(&texDesc_, 0, sizeof(texDesc_));
  texDesc_.addressMode[0] = cudaAddressModeClamp;
  texDesc_.addressMode[1] = cudaAddressModeClamp;
  texDesc_.filterMode = cudaFilterModeLinear;
  texDesc_.readMode = cudaReadModeElementType;
  texDesc_.normalizedCoords = 1;

  //First destroy the current texture object
  HANDLE_ERROR(cudaDestroyTextureObject(obstaclemap_tex_));

  //Now create the new texture object.
  HANDLE_ERROR(cudaCreateTextureObject(&obstaclemap_tex_, &resDesc_, &texDesc_, NULL) );

  //Set obstacle parameters from parameter server
  updateParams(mppi_node);
}

inline void MPPIPCCosts::updateParams_dcfg(autorally_control::PathIntegralParamsConfig config)
{
  obs_params_.obstacle_coeff = (float) config.obstacle_coefficient;
  obs_params_.obstacle_decay = (float) config.obstacle_decay;
  obs_params_.obstacle_buffer = (float) config.obstacle_buffer;
  obs_params_.obstacle_pad = (int) config.obstacle_pad;
  obsParamsToDevice();
  MPPICosts::updateParams_dcfg(config);
}

inline void MPPIPCCosts::updateParams(ros::NodeHandle mppi_node)
{
  //Read parameters from the ROS parameter server
  double obstacle_coeff, boundary_threshold, obs_boundary_threshold, obstacle_decay, obstacle_buffer, filter_length;
  int obstacle_pad;
  mppi_node.getParam("boundary_threshold", boundary_threshold);
  mppi_node.getParam("obs_boundary_threshold", obs_boundary_threshold);
  mppi_node.getParam("obstacle_coefficient", obstacle_coeff);
  mppi_node.getParam("obstacle_pad", obstacle_pad);
  mppi_node.getParam("obstacle_decay", obstacle_decay);
  mppi_node.getParam("obstacle_buffer", obstacle_buffer);
  mppi_node.getParam("filter_length", filter_length);

  //Transfer to the cost params struct
  obs_params_.boundary_threshold = (float) boundary_threshold;
  obs_params_.obs_boundary_threshold = (float) obs_boundary_threshold;
  obs_params_.obstacle_coeff = (float) obstacle_coeff;
  obs_params_.obstacle_decay = (float) obstacle_decay;
  obs_params_.obstacle_pad = (int) obstacle_pad;
  obs_params_.obstacle_buffer = (float) obstacle_buffer;
  obs_params_.filter_length = (float) filter_length;

  obsParamsToDevice();
  MPPICosts::updateParams(mppi_node);
}

inline void MPPIPCCosts::obsParamsToDevice()
{
  HANDLE_ERROR( cudaMemcpy(obs_params_d_, &obs_params_, sizeof(ObsParams), cudaMemcpyHostToDevice) );
  HANDLE_ERROR( cudaStreamSynchronize(stream_) );
}

inline void MPPIPCCosts::freeCudaMem()
{
  HANDLE_ERROR(cudaDestroyTextureObject(obstaclemap_tex_));
  HANDLE_ERROR(cudaFreeArray(obstaclemapArray_d_));
  HANDLE_ERROR(cudaFree(obs_params_d_));
  MPPICosts::freeCudaMem();
}

inline void MPPIPCCosts::resetObstacleMap()
{
  for (int idx = 0; idx < width_ * height_; idx++) {
    obstacle_costs_[idx] = 0.0;
  }

  for (int idx = 0; idx < obstacle_costs_hist_.size(); idx++) {
    obstacle_costs_hist_.pop();
  }
}

inline void MPPIPCCosts::updateObstacleMap(sensor_msgs::PointCloud2Ptr points)
{
  sensor_msgs::PointCloud2Iterator<float> points_iter_x(*points, "x");
  sensor_msgs::PointCloud2Iterator<float> points_iter_y(*points, "y");

  int x, y, x_pt, y_pt;
  int x_range_min, x_range_max, y_range_min, y_range_max;
  int x_delta_sq, y_delta, y_delta_sq;
  int obstacle_pad_sq = pow(obs_params_.obstacle_pad, 2);
  float inv_obstacle_pad = (float) 1/obs_params_.obstacle_pad;
  float factor = 1.0/min(obstacle_costs_hist_.size()+1., (float) obs_params_.filter_length);
  std::vector<float> inst_costs = std::vector<float>(width_*height_, 0.);
  int hist_size = (int) obstacle_costs_hist_.size();
  float scale;
  //std::vector<float> obstacle_costs_(width_*height_, 0.);

  if (hist_size > 0 && hist_size < obs_params_.filter_length) {
    for (int idx=0; idx < width_*height_; idx++) {
      //obstacle_costs_[idx] *= obs_params_.obstacle_decay;
      obstacle_costs_[idx] *= hist_size*factor;
    }
  }

  //Build obstacle map
  for (; points_iter_x != points_iter_x.end(); ++points_iter_x, ++points_iter_y) {
    x = int(round(*points_iter_x * ppm_ - x_min_));
    y = int(round(*points_iter_y * ppm_ - y_min_));

    //x = 3*ppm_ -x_min_;
    //y = -3*ppm_ - y_min_;

    x_range_min = min(max(x-obs_params_.obstacle_pad, 0), width_-1);
    x_range_max = min(max(x+obs_params_.obstacle_pad, 0), width_-1);

    for (int x_idx = x_range_min; x_idx <= x_range_max; x_idx++) {
      x_delta_sq = pow(x_idx-x,2);
      y_delta = int(round(sqrt(-x_delta_sq + obstacle_pad_sq)));
      y_range_min = min(max(y-y_delta, 0), height_-1);
      y_range_max = min(max(y+y_delta, 0), height_-1);

      for (int y_idx = y_range_min; y_idx <= y_range_max; y_idx++) {
        y_delta_sq = pow(y_idx-y,2);
        //scale = max((1 - sqrt((float) x_delta_sq + (float) y_delta_sq)*inv_obstacle_pad), 0.0);
        float dist = sqrt((float) x_delta_sq + (float) y_delta_sq);
        if (dist > obs_params_.obstacle_buffer) {
          scale = exp(-(dist-obs_params_.obstacle_buffer)*inv_obstacle_pad);
        } else {
          scale = 1.0;
        }

        if (inst_costs[y_idx * width_ + x_idx] < scale) {
          if (inst_costs[y_idx * width_ + x_idx] > 0) {
            obstacle_costs_[y_idx * width_ + x_idx] -= inst_costs[y_idx * width_ + x_idx]*factor;
          }
          inst_costs[y_idx * width_ + x_idx] = scale;
          obstacle_costs_[y_idx * width_ + x_idx] += scale*factor;
        }
      }
    }
  }

  if (hist_size >= obs_params_.filter_length) {
    for (int idx=0; idx < width_*height_; idx++) {
      obstacle_costs_[idx] -= obstacle_costs_hist_.front()[idx]*factor;
    }
    obstacle_costs_hist_.pop();
  }

  //Add to history queue
  obstacle_costs_hist_.push(inst_costs);

  //Transfer from CPU to GPU
  HANDLE_ERROR( cudaMemcpyToArray(obstaclemapArray_d_, 0, 0, obstacle_costs_.data(), width_*height_*sizeof(float),
                                  cudaMemcpyHostToDevice) );
  cudaStreamSynchronize(stream_);

  //Specify texture
  resDesc_.res.array.array = obstaclemapArray_d_;

  //First destroy the current texture object
  HANDLE_ERROR(cudaDestroyTextureObject(obstaclemap_tex_));

  //Now create the new texture object.
  HANDLE_ERROR(cudaCreateTextureObject(&obstaclemap_tex_, &resDesc_, &texDesc_, NULL) );
}

inline __device__ float MPPIPCCosts::getObstacleCost(float *s, int *crash)
{
  float obstacle_cost = 0;

  //Compute a transformation to get the (x,y) positions of the front and back of the car.
  float x_front = s[0] + FRONT_D * __cosf(s[2]);
  float y_front = s[1] + FRONT_D * __sinf(s[2]);
  float x_back = s[0] + BACK_D * __cosf(s[2]);
  float y_back = s[1] + BACK_D * __sinf(s[2]);
  float x_right = s[0] + RIGHT_D * __cosf(s[2] - 1.57);
  float y_right = s[1] + RIGHT_D * __sinf(s[2] - 1.57);
  float x_left = s[0] + LEFT_D * __cosf(s[2] - 1.57);
  float y_left = s[1] + LEFT_D * __sinf(s[2] - 1.57);

  float u, v, w; //Transformed coordinates

  //Cost of front of the car
  coorTransform(x_front, y_front, &u, &v, &w);
  float obstacle_cost_front = tex2D<float>(obstaclemap_tex_, u / w, v / w);

  //Cost for back of the car
  coorTransform(x_back, y_back, &u, &v, &w);
  float obstacle_cost_back = tex2D<float>(obstaclemap_tex_, u / w, v / w);

  //Cost of front of the car
  coorTransform(x_right, y_right, &u, &v, &w);
  float obstacle_cost_right = tex2D<float>(obstaclemap_tex_, u / w, v / w);

  //Cost for back of the car
  coorTransform(x_left, y_left, &u, &v, &w);
  float obstacle_cost_left = tex2D<float>(obstaclemap_tex_, u / w, v / w);

  obstacle_cost = (fabs(obstacle_cost_front) + fabs(obstacle_cost_back) + fabs(obstacle_cost_left)
                   + fabs(obstacle_cost_right)) / 4.0;
  obstacle_cost = obs_params_d_->obstacle_coeff * obstacle_cost;

  if (obstacle_cost_front >= obs_params_d_->obs_boundary_threshold
      || obstacle_cost_back >= obs_params_d_->obs_boundary_threshold) {
    crash[0] = 1;
  }

  return obstacle_cost;
}

inline cv::Mat MPPIPCCosts::getDebugDisplay(float x, float y, float heading)
{
  cv::Mat debug_img; ///< OpenCV matrix for display debug info.
  if (!debugging_){
    debugDisplayInit();
  }
  launchDebugCostKernel(x, y, heading, debug_img_width_, debug_img_height_, debug_img_ppm_,
                        costmap_tex_, obstaclemap_tex_, debug_data_d_, params_.r_c1, params_.r_c2,
                        params_.trs, stream_);
  //Now we just have to display debug_data_d_
  HANDLE_ERROR( cudaMemcpy(debug_data_, debug_data_d_, debug_img_size_*sizeof(float), cudaMemcpyDeviceToHost) );
  cudaStreamSynchronize(stream_);
  debug_img = cv::Mat(debug_img_width_*debug_img_ppm_, debug_img_height_*debug_img_ppm_, CV_32F, debug_data_);
  return debug_img;
}

//Compute the immediate running cost.
inline __device__ float MPPIPCCosts::computeCost(float *s, float *u, float *du,
                                                 float *vars, int *crash, int timestep)
{
  float cost = MPPICosts::computeCost(s, u, du, vars, crash, timestep);
  cost += getObstacleCost(s, crash);
  if (cost > 1e9 || isnan(cost)) {
    cost = 1e9;
  }
  return cost;
}

//Compute the immediate running cost.
inline __device__ float* MPPIPCCosts::computeCostTerms(float* s, float* u, float* du,
                                                     float* vars, int* crash, int timestep)
{
  float cost[7];
  cost[0] = getControlCost(u, du, vars);
  cost[1] = getTrackCost(s, crash);
  cost[2] = getObstacleCost(s, crash);
  cost[3] = getSpeedCost(s, crash);
  cost[4] = (1.0 - params_.discount)*getCrashCost(s, crash, timestep);
  cost[5] = getStabilizingCost(s);
  cost[6] = cost[0] + cost[1] + cost[2] + cost[3] + cost[4] + cost[5];
  if (cost[6] > 1e9 || isnan(cost[6])) {
    cost[6] = 1e9;
  }
  return cost;
}

inline int MPPIPCCosts::getNumCostTerms()
{
  return NUM_COST_TERMS;
}

}
