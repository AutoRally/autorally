/*
* Software License Agreement (BSD License)
* Copyright (c) 2016, Georgia Institute of Technology
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
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDseparatedseparatedING, BUT NOT LIMITED TO, THE
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
 * @file ArucoDetect3D.cpp
 * @author Henry Liao <hliao62@gatech.edu>
 * @date December 26, 2022
 * @copyright 2022 Georgia Institute of Technology
 * @brief Interface for an AutoRally chassis
 *
 * @details Implementation of ArUco tag detection with 3D depth sensing 
 ***********************************************/


#include "ArucoDetector.h"

PLUGINLIB_EXPORT_CLASS( autorally_core::ArucoDetector,  nodelet::Nodelet)

namespace autorally_core
{

AutoRallyChassis::~AutoRallyChassis()
{
    
}

void AutoRallyChassis::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nhPvt = getPrivateNodeHandle();

  detectionPub_ = nh.advertise<autorally_msgs::arucoDetection>
                     ("aruco_detections", 1);

  
  image_transport::ImageTransport it(nh);
  leftCamSub_(it, nhPvt.getParam("/left_image"), 1);
  rightCamSub_(it, nhPvt.getParam("/right_image"), 1);
  left_info_sub_ = nhPvt.subscribe(nhPvt.getParam("/left_camera_info"),1);
  right_info_sub_ = nhPvt.subscribe(nhPvt.getParam("/right_camera_info"),1);

  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, 
    sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> StereoSyncPolicy;

  message_filters::Synchronizer< StereoSyncPolicy> sync;


  sync( StereoSyncPolicy( 10 ), leftCamSub_, rightCamSub_ )
  {
    sync.registerCallback( boost::bind( videoCallback, this, _1, _2 ) );
  }
}

  void videoCallback(
    const sensor_msgs::ImageConstPtr& left_img_msg,
    const sensor_msgs::ImageConstPtr& right_img_msg
    const sensor_msgs::CameraInfo& left_info_msg
    const sensor_msgs::CameraInfo& right_info_msg
  ){

    // aruco detection for left camera
    std::vector<int> left_marker_ids;
    std::vector<std::vector<cv::Point2f>> left_marker_corners;
    std::vector<std::vector<cv::Point2f>> left_rejected_candidates;

    auto detector_parameters = cv::aruco::DetectorParameters::create();

    const auto left_cv_image = cv_bridge::toCvShare(left_image_msg, "bgr8");

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(
      left_cv_image->image, dictionary, left_marker_corners, left_marker_ids,
      detector_parameters, left_rejected_candidates);

    // aruco detection for right camera
    std::vector<int> right_marker_ids;
    std::vector<std::vector<cv::Point2f>> right_marker_corners;
    std::vector<std::vector<cv::Point2f>> right_rejected_candidates;

    auto detector_parameters = cv::aruco::DetectorParameters::create();

    const auto right_cv_image = cv_bridge::toCvShare(right_image_msg, "bgr8");

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(
      right_cv_image->image, dictionary, right_marker_corners, right_marker_ids,
      detector_parameters, right_rejected_candidates);

    // camera intrinsics
    
    const auto left_camera_matrix = cv::Mat(left_info_msg->k).reshape(1, 3);
    const auto right_camera_matrix = cv::Mat(right_info_msg->k).reshape(1, 3);
    const auto left_distortion_coefs = cv::Mat(left_info_msg->d).reshape(1, 5);
    const auto right_distortion_coefs = cv::Mat(right_info_msg->d).reshape(1, 5);

    // some calcs here for where the aruco tag actually is based on depth (will be CV in the future)


    if (detection_pub->get_subscription_count() > 0) {
      autorally_msgs::msg::arucoDetectionArray aruco_detection_array;
      aruco_detection_msg.header.frame_id = left_image_msg->header.frame_id;
      aruco_detection_msg.header.stamp = left_image_msg->header.stamp;
      const auto left_tag_count = marker_ids.size();

      for (auto tag_index = 0; tag_index < left_tag_count; tag_index++) {
        autorally_msgs::msg::arucoDetection aruco_detection_msg;
        aruco_detection_msg.id = marker_ids[tag_index];
        aruco_detection_msg.pose.position.x = translations[tag_index][0];
        aruco_detection_msg.pose.position.y = translations[tag_index][1];
        aruco_detection_msg.pose.position.z = translations[tag_index][2];

        const auto rvec = Eigen::Vector3d(
          rotations[tag_index][0], rotations[tag_index][1],
          rotations[tag_index][2]);
        Eigen::AngleAxisd angle_axis(rvec.norm(), rvec.normalized());
        Eigen::Quaterniond quaternion(angle_axis);
        aruco_detection_msg.pose.orientation.w = quaternion.w();
        aruco_detection_msg.pose.orientation.x = quaternion.x();
        aruco_detection_msg.pose.orientation.y = quaternion.y();
        aruco_detection_msg.pose.orientation.z = quaternion.z();
        arcuo_detection_array.tags.push_back(aruco_detection_msg);
      }
      tag_publisher_->publish(tag_array_msg);
    }
  }


}
