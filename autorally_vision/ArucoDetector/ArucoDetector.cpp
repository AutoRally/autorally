#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/Image.h>

#include <autorally_estimation/ArucoDetections.h>

ros::Publisher debug_img_pub;
ros::Publisher aruco_marker_pub_;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  parameters->adaptiveThreshWinSizeMax = 40;
  parameters->adaptiveThreshWinSizeStep = 4;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  // if we have markers to publish
  if(markerIds.size() > 0) {
    autorally_estimation::ArucoDetections aruco_msg;
    aruco_msg.header = msg->header;
    aruco_msg.header.frame_id = msg->header.frame_id;

    for(unsigned int i = 0; i < markerCorners.size(); i++) {
      autorally_estimation::ArucoDetection aruco_marker;
      aruco_marker.size = 0.5;
      aruco_marker.id = markerIds[i];

      // order of markers is clockwise
      for(int j = 0; j < 4; j++) {
        aruco_marker.detections.push_back(markerCorners[i][j].x);
        aruco_marker.detections.push_back(markerCorners[i][j].y);
      }
      aruco_msg.detections.push_back(aruco_marker);
    }
    aruco_marker_pub_.publish(aruco_msg);
  }

  if(debug_img_pub.getNumSubscribers() > 0) {
    cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);

    cv_bridge::CvImage cv_image;
    cv_image.image = image;
    cv_image.encoding = "bgr8";
    sensor_msgs::Image image_msg;
    cv_image.toImageMsg(image_msg);
    debug_img_pub.publish(cv_image);
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  std::string image_topic;
  pNh.param<std::string>("image_topic", image_topic, "/left_camera/image_raw");

  ros::Subscriber img_sub = nh.subscribe(image_topic, 1, imageCallback);
  std::cout << "subscribing to topic: " << image_topic << std::endl;

  debug_img_pub = nh.advertise<sensor_msgs::Image>(image_topic+"/aruco_debug", 1);
  aruco_marker_pub_ = nh.advertise<autorally_estimation::ArucoDetections>(image_topic+"/aruco_detections", 1);

  ros::spin();

  return 0;
}
