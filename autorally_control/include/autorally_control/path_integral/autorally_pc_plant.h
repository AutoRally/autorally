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
 * @file autorally_pc_plant.h
 * @author Jake Sacks <jsacks@gatech.edu>
 * @date December 11, 2018
 * @copyright 2018 Georgia Institute of Technology
 * @brief Class definition for AutorallyPCPlant class.
 ***********************************************/

#ifndef AUTORALLY_PC_PLANT_H
#define AUTORALLY_PC_PLANT_H

#include <ros/ros.h>

#include <autorally_control/path_integral/autorally_plant.h>

#include <autorally_msgs/resetObstacles.h>
#include <autorally_msgs/pathIntegralCosts.h>

namespace autorally_control {

/**
* @class AutorallyPCPlant autorally_pc_plant.h
* @brief Extension of AutorallyPlant with support for a point cloud interface
* for obstacles.
*/

class AutorallyPCPlant: public AutorallyPlant
{
public:
  /**
  * @brief Constructor for AutorallyPlant, takes the a ros node handle and initalizes
  * publishers and subscribers.
  * @param mppi_node A ros node handle.
  */
  AutorallyPCPlant(ros::NodeHandle mppi_node, bool debug_mode, int hz);

  /**
  * @brief Destructor for AutorallyPlant.
  */
  ~AutorallyPCPlant();

  /**
  * @brief Callback for obstacle_reset subscriber.
  */
  void obsResetCall(autorally_msgs::resetObstacles obs_msg);

  /**
  * @brief Callback for obstacle point cloud.
  */
  void pointsCall(sensor_msgs::PointCloud2Ptr points_msg);

  /**
  * @brief Callback for track point cloud.
  */
  void trackPointsCall(sensor_msgs::PointCloud2Ptr points_msg);

  /**
  * @brief Returns the timestamp of the last obstacle point cloud callback.
  */
  ros::Time getLastPointCloudTime();

  /**
  * @brief Returns the timestamp of the last track point cloud callback.
  */
  ros::Time getLastTrackPointCloudTime();

  /**
  * @brief Returns the timestamp of the last obstacle reset callback.
  */
  ros::Time getLastObstacleResetTime();

  /**
  * @brief Returns the current obstacle point cloud from the stereo camera
  */
  sensor_msgs::PointCloud2Ptr getPointCloud();

  /**
  * @brief Returns the current track point cloud from the stereo camera
  */
  sensor_msgs::PointCloud2Ptr getTrackPointCloud();
private:
  bool reset_obstacles_; ///< Whether or not we should reset the obstacles cost.

  ros::Time last_pc_call_; ///< Timestamp of the last point cloud callback.
  ros::Time last_track_pc_call_; ///< Timestamp of the last point cloud callback.
  ros::Time last_obs_reset_call_;

  ros::Subscriber points_sub_; ///< Subscriber to /stereo/filtered_points2.
  ros::Subscriber track_points_sub_; ///< Subscriber to /stereo/track_points2.
  ros::Subscriber obs_reset_sub_; ///< Subscriber to obstacle_reset.

  sensor_msgs::PointCloud2Ptr points_;
  sensor_msgs::PointCloud2Ptr track_points_;

  autorally_msgs::pathIntegralCosts cost_msg_; ///<pathIntegralCosts message for publishing the cost of the nominal traj
};

}

#endif //AUTORALLY_PC_PLANT_H
