//
// Created by jason on 12/2/19.
//

#include "Tracker.h"
#include <ros/ros.h>

using namespace gtsam;

Tracker::Tracker(ros::NodeHandle& nh) {
  // general config
  nh.getParam("debug", debug_);
  nh.getParam("full_history_debug", full_history_debug_);

  // ISAM config

  //nh.getParam("relinearizeThreshold", isam_parameters_.relinearizeThreshold);
  nh.getParam("relinearizeSkip", isam_parameters_.relinearizeSkip);
  nh.getParam("enablePartialRelinearizationCheck", isam_parameters_.enablePartialRelinearizationCheck);
  nh.getParam("enableRelinearization", isam_parameters_.enableRelinearization);
  nh.getParam("cacheLinearizedFactors", isam_parameters_.cacheLinearizedFactors);
  nh.getParam("findUnusedFactorSlots", isam_parameters_.findUnusedFactorSlots);
  if(debug_) {
    isam_parameters_.enableDetailedResults = true;
    isam_parameters_.evaluateNonlinearError = true;
  }
  gtsam::ISAM2DoglegParams optimizationParams;
  nh.getParam("verbose", optimizationParams.verbose);
  isam_parameters_.optimizationParams = optimizationParams;

  // set up priors of the vehicles
  std::vector<double> my_location, other_location;
  nh.getParam("my_location", my_location);
  nh.getParam("other_location", other_location);

  gtsam::Pose3 my_prior = gtsam::Pose3(gtsam::Rot3::Quaternion(my_location[3], my_location[4], my_location[5], my_location[6]),
          gtsam::Point3(my_location[0], my_location[1], my_location[2]));
  gtsam::Pose3 other_prior = gtsam::Pose3(gtsam::Rot3::Quaternion(other_location[3], other_location[4], other_location[5], other_location[6]),
          gtsam::Point3(other_location[0], other_location[1], other_location[2]));

  gtsam::Vector3 vel;
  vel << 0,0,0;

  // insert initial guesses of state
  current_state_guess_->insert(symbol_shorthand::X(index_),
                               my_prior);
  current_state_guess_->insert(symbol_shorthand::V(index_),
                               vel);

  current_state_guess_->insert(symbol_shorthand::Y(index_),
                               other_prior);
  current_state_guess_->insert(symbol_shorthand::W(index_),
                               vel);
  // tracking config
  // TODO set up noise models

  // ROS setup
  std::string other_detection_topic, pose_topic, camera_topic, other_control_callback;
  nh.getparam("other_detection_topic", other_detection_topic);
  nh.getParam("pose_topic", pose_topic);
  nh.getParam("camera_topic", camera_topic);
  nh.getParam("other_control_callback", other_control_callback);
  subs_.push_back(nh.subscribe());

}

void Tracker::run_optimize() {

}

void Tracker::self_odometry_callback(nav_msgs::Odometry::ConstPtr msg) {

}

void Tracker::other_detection_callback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg) {

}

void Tracker::other_control_callback(autorally_msgs::chassisCommand::ConstPtr cmd) {

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vehicle_tracker");
}