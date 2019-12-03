//
// Created by jason on 12/2/19.
//

#include <ros/ros.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <autorally_msgs/chassisCommand.h>

#ifndef SRC_TRACKER_H
#define SRC_TRACKER_H

struct optimization_stats {
  double optimizationTime = 0.0;
  double optimizationTimeAvg = 0.0;
  double getLandmarksTime = 0.0;
  double getLandmarksTimeAvg = 0.0;
  int optimizationIterations = 0;
  int variablesReeliminated = 0;
  int variablesRelinearized = 0;
  double errorBefore = 0.0;
  double errorAfter = 0.0;
};

class Tracker {
public:
  Tracker(ros::NodeHandle &nh);
  void run_optimize();
  void self_odometry_callback(nav_msgs::Odometry::ConstPtr msg);
  // comes in body frame
  void other_detection_callback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
  void timing_callback(sensor_msgs::Image::ConstPtr msg);
  void other_control_callback(autorally_msgs::chassisCommand::ConstPtr cmd);
private:
  // ========== GENERIC VARS =======
  std::vector<ros::Subscriber> subs_;
  ros::Publisher other_odom_pub_;
  ros::Publisher other_history_pub_;
  bool debug_ = true;
  bool full_history_debug_ = false; // TODO config

  // how long the most recent optimization took
  optimization_stats optimization_stats_;

  // ========= GRAPH GENERICS ===========
  // current graph that gets updated and cleared after each optimization
  std::shared_ptr<gtsam::NonlinearFactorGraph> current_incremental_graph_;

  gtsam::ISAM2 isam_;
  // isam parameters, set up in yaml config
  gtsam::ISAM2Params isam_parameters_;

  // maps an index to a time
  std::map<int, double> time_map_;

  // index of the current state
  int index_ = 0;

  // mutex locks
  std::mutex graph_lck_; // controls isam_ and current_incremental_graph_
  std::mutex pose_lck_; // controls the

  // Current estimate of the state to be passed into factor graph
  gtsam::Pose3 current_position_guess_;
  gtsam::Vector3 current_velocity_guess_;

  // values to store above
  /*
   * X(t) pose at time t of self
   * V(t) velocity at time t of self
   * Y(t) pose at time t of other vehicle
   * W(t) velocity at time t of other vehicle
   */
  std::shared_ptr<gtsam::Values> current_state_guess_;

  // entire history of the state
  gtsam::Values history_;

  // ========= SELF NOISE ===========
  // noise model for pose and vel
  gtsam::noiseModel::Diagonal::shared_ptr self_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr self_vel_noise_;

  // ========= OTHER NOISE ===========
  // noise model for pose and vel
  gtsam::noiseModel::Diagonal::shared_ptr other_camera_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr dynamics_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr dynamics_vel_noise_;

};


#endif //SRC_TRACKER_H
