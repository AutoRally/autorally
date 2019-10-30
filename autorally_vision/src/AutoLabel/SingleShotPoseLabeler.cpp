/*
* Software License Agreement (BSD License)
* Copyright (c) 2019, Georgia Institute of Technology
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
/**
 * @file
 * @author Jason Gibson <jgibson37@gatech.edu>
 * @date October 28, 2019
 * @copyright 2019 Georgia Institute of Technology
 * @brief Auto Label Tool
 *
 * @details generic wrapper for image auto labeling
 **/
#include <AutoLabel/SingleShotPoseLabeler.h>
//#include <autorally_vision/AutoLabel/SingleShotPoseLabeler.h>

namespace autorally_vision {

SingleShotPoseLabeler::SingleShotPoseLabeler(ros::NodeHandle nh) {
  // TODO load all valid params
  std::string mode = "";
  nh.getParam("test", mode);
  // load in what compute box we want, camera matricies of each chassis
  // load in list of compute box to track with ground truth
  std::vector<std::string> other_chassis_names;
  nh.getParam("other_compute_box_names", other_chassis_names);
  for(std::string compute_box_name : other_chassis_names) {
    // get the calibration values namespaced
    std::vector<double> cam_mat;
    nh.getParam("cam_mat", cam_mat);
    CameraParameters cameraParameters;
    cameraParameters.K = (cv::Mat_<double>(3,4) <<cam_mat[0], 0.0, cam_mat[2], 0.0,
                          0.0, cam_mat[1], cam_mat[3], 0.0,
                          0.0, 0.0, 1.0, 0.0);
    std::vector<double> cam_R;
    nh.getParam("cam_R", cam_R);
    cameraParameters.R = (cv::Mat_<double>(3,3) << cam_R[0], cam_R[1], cam_R[2],
                          cam_R[3], cam_R[4], cam_R[5],
                          cam_R[6], cam_R[7], cam_R[8]);

    std::vector<double> cam_translation;
    nh.getParam("cam_translation", cam_translation);
    cameraParameters.translation = (cv::Mat_<double>(3, 1) <<
            cam_translation[0], cam_translation[1], cam_translation[2]);
    cameraParamMap_.insert(std::make_pair(compute_box_name, cameraParameters));
  }
  if(mode == "test") {
    // if we are testing then set up subscribers
  } else if("label") {
    std::string output_dir;
    nh.getParam("output_dir", output_dir);

    // if we are labeling read in bag files and output labels
    rosbag::Bag my_bag;
    // full path
    std::string my_bag_file;
    nh.getParam("my_bag_file", my_bag_file);
    //  load my bag file
    my_bag.open(my_bag_file);
    std::vector<std::string> my_topics = {"/pose_estimate", "/left_camera/image_color/compressed"};
    rosbag::View my_bag_view(my_bag, rosbag::TopicQuery(my_topics));
    rosbag::View::iterator my_it = my_bag_view.begin();

    // load the other bag files
    std::map<std::string, std::pair<rosbag::View::iterator, rosbag::View::iterator>> other_bags;
    std::vector<std::string> other_bag_files;
    nh.getParam("other_bag_files", other_bag_files);
    for(const std::string& filepath : other_bag_files) {
      int location = filepath.find_last_of("/", 0, std::string::npos);
      std::string bag_file_name = filepath.substr(location);
      int first = bag_file_name.find("_", 0, std::string::npos);
      std::string chassis_name = bag_file_name.substr(0, first);
      std::string compute_box_name = bag_file_name.substr(first,
              bag_file_name.find("_", first, std::string::npos));
      rosbag::Bag bag = rosbag::Bag(filepath);
      std::vector<std::string> topics = {"/pose_estimate"};
      rosbag::View view(bag, rosbag::TopicQuery(topics));
      other_bags.insert(std::make_pair(compute_box_name, std::make_pair(view.begin(), view.end())));
    }

    // what image number we are on to correlate txt with image
    int counter = 0;
    // while there is more to search through my bag file
    while(my_it != my_bag_view.end()) {
      // find the next image, <image, time>
      auto image_result = findNextImage(my_it, my_bag_view.end());
      // save out the image
      cv::imwrite(output_dir+"/test_"+std::to_string(counter)+".png", image_result.mat);
      // create text file to write out to
      std::ofstream outfile;
      outfile.open(output_dir+"/test_"+std::to_string(counter)+".txt");

      // find the poses of the other vehicles at given time
      std::map<std::string, nav_msgs::Odometry> poses;
      for(auto it = other_bags.begin(); it != other_bags.end(); it++) {
        nav_msgs::Odometry odom_msg = findNextInterpolatedPose(it->second.first, it->second.second, image_result.time);
        // if -1 time that means it could not interpolate and we should ignore this
        if(odom_msg.header.stamp.toSec() == -1) {
          // ignore that image
          continue;
        }
        // get 3D bounding box in body frame
        BoundingBox3D bbox_3D_body = getBoundingBoxBody(image_result.interp_pose, odom_msg);
        // get the 2D projection of the 3D bounding box
        FlattenedBoundingBox3D bbox_2D = get2DProjection(bbox_3D_body, it->first);
        // write out line to text file
        writeOutData(bbox_2D, outfile);
        // write out debug image, modifies the input image
        writeOutDebug(bbox_2D, image_result.mat);
      }
      cv::imwrite(output_dir+"/test_debug_"+std::to_string(counter)+".png", image_result.mat);
      outfile.close();
      ++counter;
    }


  } else {
    ROS_ERROR("INVALID mode selected");
  }
}

  BoundingBox3D SingleShotPoseLabeler::getBoundingBoxWorld(const nav_msgs::Odometry& their_pose) {
    BoundingBox3D result;
    double x = their_pose.pose.pose.position.x;
    double y = their_pose.pose.pose.position.y;
    // used to be fixed
    double z = their_pose.pose.pose.orientation.z;     //Z position fluctuates a lot
    double qw = their_pose.pose.pose.orientation.w;
    double qx = their_pose.pose.pose.orientation.x;
    double qy = their_pose.pose.pose.orientation.y;
    double qz = their_pose.pose.pose.orientation.z;
    tf::Quaternion q(qx, qy, qz, qw);   // Create quaternion object and get RPY
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double car_front_x = x + car_length_ * cos(yaw);  // Center point of front of vehicle in world frame
    double car_front_y = y + car_length_ * sin(yaw);
    double car_back_x = x - car_length_ * cos(yaw);  // Center point of back of vehicle in world frame
    double car_back_y = y - car_length_ * sin(yaw);
    result.centroid = Point3D(x, y, z);
    result.bounding_box[0] = Point3D(car_front_x - car_width_ * sin(yaw), car_front_y + car_width_ * cos(yaw), z); //Front top left
    result.bounding_box[1] = Point3D(car_front_x - car_width_ * sin(yaw), car_front_y + car_width_ * cos(yaw), 0); //Front bottom left
    result.bounding_box[2] = Point3D(car_front_x + car_width_ * sin(yaw), car_front_y - car_width_ * cos(yaw), z); //Front top right
    result.bounding_box[3] = Point3D(car_front_x + car_width_ * sin(yaw), car_front_y - car_width_ * cos(yaw), 0); //Front bottom right
    result.bounding_box[4] = Point3D(car_back_x - car_width_ * sin(yaw), car_back_y + car_width_ * cos(yaw), z); //Back top left
    result.bounding_box[5] = Point3D(car_back_x - car_width_ * sin(yaw), car_back_y + car_width_ * cos(yaw), 0); //Back bottom left
    result.bounding_box[6] = Point3D(car_back_x + car_width_ * sin(yaw), car_back_y - car_width_ * cos(yaw), z); //Back top right
    result.bounding_box[7] = Point3D(car_back_x + car_width_ * sin(yaw), car_back_y - car_width_ * cos(yaw), 0); //Back bottom right
    return result;
  }

  BoundingBox3D SingleShotPoseLabeler::getBoundingBoxBody(const nav_msgs::Odometry& my_pose, const nav_msgs::Odometry& their_pose) {
    BoundingBox3D result;
    // get 3D bounding box in world frame
    BoundingBox3D bbox_3D_world = getBoundingBoxWorld(their_pose);
    for(Point3D point : bbox_3D_world.bounding_box) {

    }
    return result;
  }

  FlattenedBoundingBox3D SingleShotPoseLabeler::get2DProjection(const BoundingBox3D& bbox, const std::string& compute_box_name) {
    FlattenedBoundingBox3D result;
    return result;
  }

  void SingleShotPoseLabeler::writeOutData(const FlattenedBoundingBox3D& bbox, std::ofstream& stream) {
    // TODO write out
    stream << "0," << bbox.centroid.x << ", "
            << bbox.centroid.y << ", "
            << bbox.bounding_box[0].x << ", "
            << bbox.bounding_box[0].y << ", "
            << bbox.bounding_box[1].x << ", "
            << bbox.bounding_box[1].y << ", "
            << bbox.bounding_box[2].x << ", "
            << bbox.bounding_box[2].y << ", "
            << bbox.bounding_box[3].x << ", "
            << bbox.bounding_box[3].y << ", "
            << bbox.bounding_box[4].x << ", "
            << bbox.bounding_box[4].y << ", "
            << bbox.bounding_box[5].x << ", "
            << bbox.bounding_box[5].y << ", "
            << bbox.bounding_box[6].x << ", "
            << bbox.bounding_box[6].y << ", "
            << bbox.bounding_box[7].x << ", "
            << bbox.bounding_box[7].y << ", "
            << bbox.x_range << ", " << bbox.y_range << "\n";
  }

  void SingleShotPoseLabeler::writeOutDebug(const FlattenedBoundingBox3D& bbox, cv::Mat& img) {
    cv::circle(img, cv::Point(bbox.centroid.x, bbox.centroid.y), 10, CV_RGB(0,255,0));// Draw a circle at each corner
    for(Point2D point : bbox.bounding_box) {
      if(point.x > 0 && point.x < img.rows && point.y > 0 && point.y < img.cols) {
        cv::circle(img, cv::Point(point.x, point.y), 10, CV_RGB(255,0,0));// Draw a circle at each corner
      }
    }
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "autoLabeler");
  ros::NodeHandle pNh("~");
  autorally_vision::SingleShotPoseLabeler singleShotPoseLabeler(pNh);
  ros::spin();
}
