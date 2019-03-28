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
* @file costs_pc.cuh
* @author Jake Sacks <jsacks@gatech.edu>
* @date December 11, 2018
* @copyright 2018 Georgia Institute of Technology
* @brief MPPIPCCosts class definition
***********************************************/

#ifndef MPPI_PC_COSTS_CUH_
#define MPPI_PC_COSTS_CUH_

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include "costs.cuh"

#include <vector>
#include <queue>

namespace autorally_control {

/**
* @class MPPICosts mppi_pc_costs.cuh
* @brief Derived class which extends MPPICosts to include obstacles from point clouds
*/
class MPPIPCCosts: public MPPICosts
{
public:

  static const int NUM_COST_TERMS = 7;

  //Struct for holding various cost parameters.
  typedef struct
  {
    float obstacle_coeff;
    float boundary_threshold;
    float obs_boundary_threshold;
    float obstacle_decay;
    int obstacle_pad;
    int obstacle_buffer;
    float filter_length;
  } ObsParams;

  ObsParams obs_params_; ///< Struct for obstacle parameters.

  /**
  * @brief Constructor for when loading cost grid and transform from a file.
  * @param mppi_node Node handle to the controller ROS node.
  */
  MPPIPCCosts(ros::NodeHandle mppi_node);

  /**
  * @brief Updates the cost parameters used by MPPI
  * @param config Dynamic reconfigure variables
  */
  void updateParams_dcfg(autorally_control::PathIntegralParamsConfig config);

  /*
  * @brief Updates cost parameters by reading from the rosparam server
  * @params mppi_node Node handle to the controller ROS node.
  */
  void updateParams(ros::NodeHandle mppi_node);

  /*
  * @brief Copy the obs_params_ struct to the gpu.
  */
  void obsParamsToDevice();

  /*
  * @brief Use point cloud to update the obstacle map
  * @param points
  */
  void updateObstacleMap(sensor_msgs::PointCloud2Ptr points);

  /*
  * @brief Resets the obstacle costmap in case the decay is slow
  */
  void resetObstacleMap();

  /*
  * @brief Free cuda variables/memory.
  */
  void freeCudaMem();

  /*
  * @brief Compute the current obstacle cost based on the obstaclemap.
  */
  __device__ float getObstacleCost(float* s, int* crash);

  /*
  * @brief Return the number of cost terms
  */
  int getNumCostTerms();

  /*
  * @brief Display the debug view centered around x and y.
  * @param x float representing the current x-coordinate
  * @param y float representing the current y-coordinate
  */
  cv::Mat getDebugDisplay(float x, float y, float heading);
  cv::Mat getCostmapDisplay(float x, float y, float heading);

  /*
  * @brief Compute all of the individual cost terms and adds them together.
  */
  __device__ float computeCost(float* s, float* u, float* du, float* vars, int* crash, int t);
  __device__ float* computeCostTerms(float* s, float* u, float* du, float* vars, int* crash, int t);

protected:
  //Constant variables
  const int FILTER_LENGTH = 10; ///< Filter length for point cloud moving average
  const float LEFT_D = -0.25; ///< Distance from GPS receiver to left side of car.
  const float RIGHT_D = 0.25; ///< Distance from GPS receiver to right side of car.

  //Primary variables
  ObsParams* obs_params_d_; ///< Device side copy of params_.
  cudaTextureObject_t obstaclemap_tex_; ///< Cuda texture object.
  std::vector<float> obstacle_costs_;
  std::queue<std::vector<float>> obstacle_costs_hist_;
  cudaArray *obstaclemapArray_d_; ///< Cuda array for texture binding of obstacle map.
  cudaChannelFormatDesc obsChannelDesc_;

  struct cudaResourceDesc resDesc_;
  struct cudaTextureDesc texDesc_;
};

}

#include "costs_pc.cu"

#endif //MPPI_PC_COSTS_CUH_
