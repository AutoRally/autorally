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

SingleShotPoseLabeler::SingleShotPoseLabeler(ros::NodeHandle nh, std::string mode) {
  // TODO load all valid params
  nh.getParam("half_width", car_width_);
  nh.getParam("half_length", car_length_);
  nh.getParam("half_height", car_height_);


  // load in what compute box we want, camera matricies of each chassis
  // load in list of compute box to track with ground truth
  std::vector<std::string> compute_box_names;
  nh.getParam("compute_box_names", compute_box_names);
  for(std::string compute_box_name : compute_box_names) {
    ROS_INFO_STREAM("getting parameters for " << compute_box_name);
    // get the calibration values namespaced
    std::vector<double> cam_mat;
    nh.getParam(compute_box_name+"/cam_mat", cam_mat);
    CameraParameters cameraParameters;
    cameraParameters.K = (cv::Mat_<double>(3,4) << cam_mat[0], 0.0, cam_mat[2], 0.0,
                          0.0, cam_mat[1], cam_mat[3], 0.0,
                          0.0, 0.0, 1.0, 0.0);
    std::vector<double> cam_R;
    nh.getParam(compute_box_name+"/cam_R", cam_R);
    cameraParameters.R = (cv::Mat_<double>(3,3) << cam_R[0], cam_R[1], cam_R[2],
                          cam_R[3], cam_R[4], cam_R[5],
                          cam_R[6], cam_R[7], cam_R[8]);

    std::vector<double> cam_translation;
    nh.getParam(compute_box_name+"/cam_translation", cam_translation);
    cameraParameters.translation = (cv::Mat_<double>(3, 1) <<
            cam_translation[0], cam_translation[1], cam_translation[2]);
    cameraParamMap_.insert(std::make_pair(compute_box_name, cameraParameters));
    ROS_INFO_STREAM("K: \n" << cameraParameters.K);
  }
  if(mode == "test") {
    // if we are testing then set up subscribers
  } else if("label") {
    std::string output_dir;
    nh.getParam("output_dir", output_dir);

    double start_time = 0;
    nh.getParam("start_time", start_time);
    nh.getParam("my_z", my_z_);
    nh.getParam("their_z", their_z_);

    // if we are labeling read in bag files and output labels
    rosbag::Bag my_bag;
    // full path
    std::string my_bag_file;
    nh.getParam("my_bag_file", my_bag_file);
    //  load my bag file
    ROS_INFO_STREAM("Opening bag file for camera images: " << my_bag_file);
    my_bag.open(my_bag_file, rosbag::bagmode::Read);
    std::vector<std::string> my_topics = {"/pose_estimate", "/left_camera/image_color/compressed"};
    rosbag::View my_bag_view(my_bag, rosbag::TopicQuery(my_topics), ros::Time(start_time));
    rosbag::View::iterator my_it = my_bag_view.begin();

    // load the other bag files
    std::map<std::string, ros::Time> other_bags;
    std::vector<std::string> other_bag_files;
    nh.getParam("other_bag_files", other_bag_files);
    std::map<std::string, std::shared_ptr<rosbag::Bag>> bags;
    std::vector<std::shared_ptr<rosbag::View>> views(other_bag_files.size());
    std::vector<std::string> topics = {"/pose_estimate"};
    for(const std::string& filepath : other_bag_files) {
      ROS_INFO_STREAM("Opening bag file for other vehicle poses: " << filepath);
      int location = filepath.find_last_of("/", std::string::npos);
      std::string bag_file_name = filepath.substr(location+1, std::string::npos);
      ROS_INFO_STREAM("bag file name: " << bag_file_name);
      int first = bag_file_name.find("_", 0);
      std::string chassis_name = bag_file_name.substr(0, first);
      ROS_INFO_STREAM("found bag file with chassis_name: " << chassis_name);
      ROS_INFO_STREAM("positoin: " << bag_file_name.find("_", first+1));
      std::string compute_box_name = bag_file_name.substr(first+1,
              bag_file_name.find("_", first+1) - (chassis_name.size()+1));
      ROS_INFO_STREAM("found bag file with compute_box_name: " << compute_box_name);
      bags[compute_box_name] = std::make_shared<rosbag::Bag>(filepath, rosbag::bagmode::Read);
      other_bags.insert(std::make_pair(compute_box_name, ros::Time(start_time)));
    }

    // what image number we are on to correlate txt with image
    int counter = 0;
    // while there is more to search through my bag file
    while(my_it != my_bag_view.end() && ros::ok()) {
      ROS_INFO_STREAM("main loop counter = " << counter << "\n=============================");
      // find the next image, <image, time>
      auto image_result = findNextImage(my_it, my_bag_view.end());
      if(image_result.time.toSec() == 0) {
        ROS_INFO_STREAM("could not find anymore images");
        break;
      }
      // save out the image
      cv::imwrite(output_dir+"/test_"+std::to_string(counter)+".png", image_result.mat);
      // create text file to write out to
      std::ofstream outfile;
      outfile.open(output_dir+"/test_"+std::to_string(counter)+".txt");

      // find the poses of the other vehicles at given time
      std::map<std::string, nav_msgs::Odometry> poses;
      for(auto it = other_bags.begin(); it != other_bags.end(); it++) {

        rosbag::View view(*bags[it->first], rosbag::TopicQuery(topics), it->second);
        auto temp_it = view.begin();
        nav_msgs::Odometry odom_msg = findNextInterpolatedPose(temp_it, view.end(), image_result.time);
        it->second = odom_msg.header.stamp;
        // if -1 time that means it could not interpolate and we should ignore this
        if(odom_msg.header.stamp.toSec() == 0) {
          // ignore that image
          continue;
        }
        ROS_INFO_STREAM("my interpolated pose " << image_result.interp_pose.pose.pose);
        ROS_INFO_STREAM("their interpolated pose " << odom_msg.pose.pose);
        // get the 2D projection of the 3D bounding box
        FlattenedBoundingBox3D bbox_2D = getBoundingBoxBody(image_result.interp_pose, odom_msg);
        // write out line to text file
        writeOutData(bbox_2D, outfile);
        // write out debug image, modifies the input image
        writeOutDebug(bbox_2D, image_result.mat);
        ros::spinOnce();
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
    //double z = their_pose.pose.pose.orientation.z;     //Z position fluctuates a lot
    double z = their_z_;
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
    result.centroid = (cv::Mat_<double>(4,1) << x, y, z, 1);
    result.bounding_box[0] = (cv::Mat_<double>(4,1) << car_front_x - car_width_ * sin(yaw), car_front_y + car_width_ * cos(yaw), z + car_height_, 1); //Front top left
    result.bounding_box[1] = (cv::Mat_<double>(4,1) << car_front_x - car_width_ * sin(yaw), car_front_y + car_width_ * cos(yaw), z - car_height_, 1); //Front bottom left
    result.bounding_box[2] = (cv::Mat_<double>(4,1) << car_front_x + car_width_ * sin(yaw), car_front_y - car_width_ * cos(yaw), z + car_height_, 1); //Front top right
    result.bounding_box[3] = (cv::Mat_<double>(4,1) << car_front_x + car_width_ * sin(yaw), car_front_y - car_width_ * cos(yaw), z - car_height_, 1); //Front bottom right
    result.bounding_box[4] = (cv::Mat_<double>(4,1) << car_back_x - car_width_ * sin(yaw), car_back_y + car_width_ * cos(yaw), z + car_height_, 1); //Back top left
    result.bounding_box[5] = (cv::Mat_<double>(4,1) << car_back_x - car_width_ * sin(yaw), car_back_y + car_width_ * cos(yaw), z - car_height_, 1); //Back bottom left
    result.bounding_box[6] = (cv::Mat_<double>(4,1) << car_back_x + car_width_ * sin(yaw), car_back_y - car_width_ * cos(yaw), z + car_height_, 1); //Back top right
    result.bounding_box[7] = (cv::Mat_<double>(4,1) << car_back_x + car_width_ * sin(yaw), car_back_y - car_width_ * cos(yaw), z - car_height_, 1); //Back bottom right
    return result;
  }

  FlattenedBoundingBox3D SingleShotPoseLabeler::getBoundingBoxBody(const nav_msgs::Odometry& my_pose, const nav_msgs::Odometry& their_pose) {
    FlattenedBoundingBox3D result;
    // get 3D bounding box in world frame
    BoundingBox3D bbox_3D_world = getBoundingBoxWorld(their_pose);

    ROS_INFO_STREAM("centoriod: " << bbox_3D_world.centroid);

    cv::Mat world_to_cat; // Transformation matrix from world frame to Cat's frame
    double qw = my_pose.pose.pose.orientation.w;
    double qx = my_pose.pose.pose.orientation.x;
    double qy = my_pose.pose.pose.orientation.y;
    double qz = my_pose.pose.pose.orientation.z;
    cv::Mat R = (cv::Mat_<double>(3,3) << 1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw,      //Get 3x3 rotation matrix
            2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw,
            2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy);
    //cv::Mat R = (cv::Mat_<double>(3,3) << 0, -1, 0, 0, 0, -1, 1, 0, 0);                               // Ideal rotation matrix
    cv::Mat T = (cv::Mat_<double>(3,1) << my_pose.pose.pose.position.x,                            //Get 3x1 translation vector
            my_pose.pose.pose.position.y,
            my_z_);   //Z fluctuates a lot
    hconcat(R.t(), -1 * R.t() * T, world_to_cat);   // Create projection matrix from world frame to Cat's frame
    cv::Mat brow = (cv::Mat_<double>(1,4) << 0, 0, 0, 1);   // Homogenize
    vconcat(world_to_cat, brow, world_to_cat);

    // TODO make generic
    CameraParameters cam_param = cameraParamMap_["autorally3"];

    cv::Mat cat_to_im_R = (cv::Mat_<double>(3,3) << -1, 0, 0, 0, -1, 0, 0, 0, 1) *       // Flip axes to match image
                          (cv::Mat_<double>(3,3) <<
                                  cam_param.R.at<double>(0,0), cam_param.R.at<double>(0,1) ,cam_param.R.at<double>(0,2),
                                  cam_param.R.at<double>(1,0), cam_param.R.at<double>(1,1) ,cam_param.R.at<double>(1,2),
                                  cam_param.R.at<double>(2,0), cam_param.R.at<double>(2,1) ,cam_param.R.at<double>(2,2));
    cv::Mat cat_to_im_T = (cv::Mat_<double>(3,1) <<
            cam_param.translation.at<double>(0,0), cam_param.translation.at<double>(1,0) ,cam_param.translation.at<double>(2,0));
    cv::Mat_<double> cat_to_im;
    hconcat(cat_to_im_R, cat_to_im_T, cat_to_im);
    vconcat(cat_to_im, brow, cat_to_im);

    //ROS_INFO_STREAM("K: \n" << cam_param.K);
    //ROS_INFO_STREAM("cat_to_im: \n" << cat_to_im);
    //ROS_INFO_STREAM("world to cat: \n" << world_to_cat);

    cv::Mat point_to_plot = cam_param.K * cat_to_im * world_to_cat * bbox_3D_world.centroid;

    double x = point_to_plot.at<double>(0,0) / point_to_plot.at<double>(2,0);
    double y = point_to_plot.at<double>(1,0) / point_to_plot.at<double>(2,0);

    result.centroid = cv::Point2f(x, y);

    //ROS_INFO_STREAM("centroid projected: " << result.centroid);

    for(int i = 0; i < bbox_3D_world.bounding_box.size(); i++) {
      point_to_plot = cam_param.K * cat_to_im * world_to_cat * bbox_3D_world.bounding_box[i];

      x = point_to_plot.at<double>(0,0) / point_to_plot.at<double>(2,0);
      y = point_to_plot.at<double>(1,0) / point_to_plot.at<double>(2,0);

      result.bounding_box[i] = cv::Point2f(x, y);
    }
    return result;
  }

  FlattenedBoundingBox3D SingleShotPoseLabeler::get2DProjection(const BoundingBox3D& bbox, const std::string& compute_box_name) {
    FlattenedBoundingBox3D result;
    return result;
  }

  void SingleShotPoseLabeler::writeOutData(const FlattenedBoundingBox3D& bbox, std::ofstream& stream) {
  // TODO write something else out if there is no vehicle in image
    stream << "0," << bbox.centroid.x <<  ", "
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
    cv::circle(img, bbox.centroid, 10, CV_RGB(0,255,0));// Draw a circle at each corner
    for(cv::Point2f point : bbox.bounding_box) {
      if(point.x > 0 && point.x < img.rows && point.y > 0 && point.y < img.cols) {
        cv::circle(img, point, 10, CV_RGB(255,0,0));// Draw a circle at each corner
      }
    }
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "autoLabeler");
  ros::NodeHandle pNh("~");
  std::string mode = "";
  pNh.getParam("mode", mode);
  autorally_vision::SingleShotPoseLabeler singleShotPoseLabeler(pNh, mode);
  if(mode != "label") {
    ros::spin();
  }
}
