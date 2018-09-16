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
* @file costs.cuh
* @author Grady Williams <gradyrw@gmail.com>
* @date May 24, 2017
* @copyright 2017 Georgia Institute of Technology
* @brief MPPICosts class definition
***********************************************/
#ifndef MPPI_COSTS_CUH_
#define MPPI_COSTS_CUH_

#include "managed.cuh"
#include "param_getter.h"
#include "cnpy.h"
#include <autorally_control/PathIntegralParamsConfig.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <cuda_runtime.h>

namespace autorally_control {

/**
* @class MPPICosts mppi_costs.cuh
* @brief Standard cost funtion implementation for the MPPIController 
*  
* Maintains a collection of variables and functions which are needed for
* computing costs in the mppi framework. These include host side functions
* which initialize/update variables, host side functions which handle the
* transfer of memory from the CPU to the GPU, and device functions which perform
* the actual cost computations. This class can be inherited by more specialized
* cost function implementations.
*/
class MPPICosts: public Managed
{
public:
 
  /**
  * @struct CostParams mppi_costs.cuh
  * @brief Collection of variables that are needed for cost functions computation.
  */
  typedef struct
  {
    float desired_speed;
    float speed_coeff;
    float track_coeff;
    float max_slip_ang;
    float slip_penalty;
    float track_slop;
    float crash_coeff;
    float steering_coeff;
    float throttle_coeff;
    float boundary_threshold;
    float discount;
    int num_timesteps;
    int grid_res;
    float3 r_c1;
    float3 r_c2;
    float3 trs;
  } CostParams;

  CostParams params_; ///< Struct for cost parameters.

  /**
  * @brief Simple constructor for MppiCost. 
  * @param width The width (# elements across one row) of the costmap.
  * @param height The height (# elements across one column) of the costmap.
  */
  MPPICosts(int width, int height);

  /**
  * @brief Constructor for when loading cost grid and transform from a file specified at launch time.
  * @param nh The nodehandle currently being used.
  */
  MPPICosts(ros::NodeHandle nh);

  /**
  * @brief Allocates memory to cuda array which is bound to a texture. 
  *
  * Allocates an array using the special cudaMallocArray function.
  * The size of the array allocated by this function is determined based on 
  * the width and height of the costmap. This function is called by both constructors.
  */
  void allocateTexMem();

  /**
  * @brief Updates the cost parameters used by MPPI
  * @param config Dynamic reconfigure variables 
  *
  * It is assumed that dynamic reconfigure variables are received somewhere else in the system.
  * This function allows the dynamics reconfigure variables to be passed through directly to
  * the cost function. 
  */
  void updateParams_dcfg(autorally_control::PathIntegralParamsConfig config);

  /**
  * @brief Initializes the host side costmap to all zeros.
  *
  * Initializes a float4 vector to the correct width and height and sets every value to zero.
  * This function is called by both constructors.
  */
  void initCostmap();

  void costmapToTexture(float* costmap, int channel = 0);

  /**
  * @brief Binds the member variable costmap to a CUDA texture.
  * 
  */
  void costmapToTexture();
  
  /*
  * @brief Updates cost parameters by reading from the rosparam server
  * @params mppi_node Node handle to the controller ROS node. 
  */
  void updateParams(ros::NodeHandle mppi_node);


  //void updateParams_dcfg(autorally_control::PathIntegralParamsConfig &config, int lvl);
  /*
  * @brief Updates the current costmap coordinate transform.
  * @param h Matrix representing a transform from world to (offset) costmap coordinates.
  * @param trs Array representing the offset. 
  */
  void updateTransform(Eigen::MatrixXf h, Eigen::ArrayXf trs);

  /*
  * @brief Loads track data from a file.
  * @param C-string representing the path to the costmap data file.
  * @param h Matrix representing a transform from world to (offset) costmap coordinates.
  * @param trs Array representing the offset.
  */
  std::vector<float4> loadTrackData(std::string map_path, Eigen::Matrix3f &R, Eigen::Array3f &trs);

  //std::vector<float4> loadTrackData(const char* costmap_path, Eigen::Matrix3f &R, Eigen::Array3f &trs);

  /*
  * @brief Copy the params_ struct to the gpu.
  */
  void paramsToDevice();

  /*
  * @brief TODO: Return some useful information about the cost
  */
  void getCostInfo();

  /*
  * @brief Return what the desired speed is set to.
  */
  float getDesiredSpeed();

  /*
  * @brief Sets the desired speed of the vehicle.
  * @param desired_speed The desired speed.
  */
  void setDesiredSpeed(float desired_speed);

  /*
  *@brief Initializes the debug window for a default 20x20 meter window.
  */
  void debugDisplayInit();

  /*
  * @brief Initialize and allocate memory for debug window display
  */
  void debugDisplayInit(int width_m, int height_m, int ppm);

  /*
  * @brief Display the debug view centered around x and y.
  * @param x float representing the current x-coordinate
  * @param y float representing the current y-coordinate 
  */
  cv::Mat getDebugDisplay(float x, float y, float heading);

  void updateCostmap(std::vector<int> description, std::vector<float> data);

  void updateObstacles(std::vector<int> description, std::vector<float> data);

  /*
  * @brief Free cuda variables/memory.
  */
  void freeCudaMem();

  /*
  * @brief Returns whether or not the vehicle has crashed or not
  */
  __host__ __device__ void getCrash(float* state, int* crash);

  /*
  * @brief Compute the control cost
  */
  __host__ __device__ float getControlCost(float* u, float* du, float* vars);

  /*
  * @brief Compute the cost for achieving a desired speed
  */
  __host__ __device__ float getSpeedCost(float* s, int* crash);

  /*
  * @brief Compute a penalty term for crashing
  */
  __host__ __device__ float getCrashCost(float* s, int* crash, int num_timestep);

  /*
  * @brief Compute some cost terms that help stabilize the car.
  */
  __host__ __device__ float getStabilizingCost(float* s);

  /*
  * @brief Compute a coordinate transform going from world to costmap coordinates.
  */
  __host__ __device__ void coorTransform(float x, float y, float* u, float* v, float* w);

  /*
  * @brief Compute the current track cost based on the costmap.
  */
  __device__ float getTrackCost(float* s, int* crash);

  /*
  * @brief Compute all of the individual cost terms and adds them together.
  */
  __device__ float computeCost(float* s, float* u, float* du, float* vars, int* crash, int t);

  __device__ float terminalCost(float* s);

protected:

  //Constant variables
  const float FRONT_D = 0.5; ///< Distance from GPS receiver to front of car.
  const float BACK_D = -0.5; ///< Distance from GPS receiver to back of car.
  const float DISCOUNT = 0.9; ///< Discount on the crashing cost coefficient

  bool l1_cost_; //Whether to use L1 speed cost (if false it is L2)

  //Primary variables
  int width_, height_; ///< Width and height of costmap.
  CostParams* params_d_; ///< Device side copy of params_.
  cudaArray *costmapArray_d_; ///< Cuda array for texture binding.
  cudaChannelFormatDesc channelDesc_; ///< Cuda texture channel description.
  cudaTextureObject_t costmap_tex_; ///< Cuda texture object.
  //float4* costmap_;
  std::vector<float4> track_costs_;

  //Debugging variables
  float* debug_data_; ///< Host array for holding debug info.
  float* debug_data_d_; ///< Device array for holding debug info.
  int debug_img_width_; ///Width (in meters) of area imaged by debug view.
  int debug_img_height_; ///< Height (in meters) of area imaged by debug view.
  int debug_img_ppm_; ///< Pixels per meter for resolution of debug view.
  int debug_img_size_; ///< Number of pixels in the debug image.
  bool debugging_; ///< Indicator for if we're in debugging mode
};

}

#include "costs.cu"

#endif /*MPPI_COST_CUH*/
