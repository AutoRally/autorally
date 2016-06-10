#include "ImageRepublisher.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

PLUGINLIB_DECLARE_CLASS(autorally_core, ImageRepublisher, autorally_core::ImageRepublisher, nodelet::Nodelet)

namespace autorally_core
{

ImageRepublisher::ImageRepublisher()
  : it(ros::NodeHandle())
{
}

void ImageRepublisher::onInit()
{
  nh = getNodeHandle();
  pnh = getPrivateNodeHandle();
  it = image_transport::ImageTransport(nh);

  double fps;
  pnh.param("fps", fps, 24.0);
  secondsPerFrame = 1.0 / fps;
  pnh.param("resizeHeight", resizeHeight, 480);

  pub = it.advertise("camera/image_display", 10);

  sub = it.subscribe("camera/image_raw", 1, &ImageRepublisher::imageCallback, this);
}

void ImageRepublisher::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // Don't bother computing if no nodes are subscribed.
  if(sub.getNumPublishers() == 0)
  {
    return;
  }

  // Limit frame rate
  if( (ros::Time::now() - lastFrameTime).toSec() < secondsPerFrame )
  {
    return;
  }
  lastFrameTime = ros::Time::now();

  std::string encoding = msg->encoding;

  cv::Mat dst;

  if(encoding == "mono8")
  {
      dst = cv::Mat(msg->height, msg->width, CV_8UC1);
      copyVectorToMat(msg->data, dst);
  }
  else if(encoding == "mono16")
  {
      std::vector<unsigned char> src_data = msg->data;
      dst = cv::Mat(msg->height, msg->width, CV_8UC1);
      uchar *data = dst.data;
      for(size_t i = 0; i < msg->height * msg->width * 2; i += 2)
      {
          *(data++) = ( 255 * (src_data[i] << 8 | src_data[i+1]) ) / 65536;
      }
  }
  else if(encoding == "bayer_bggr8") // raw8
  {
      cv::Mat bayer(msg->height, msg->width, CV_8UC1);
      copyVectorToMat(msg->data, bayer);
      dst = cv::Mat(msg->height, msg->width, CV_8UC3);
      cv::cvtColor(bayer, dst, CV_BayerRG2RGB);
  }
  else if(encoding == "bayer_bggr16") // raw16
  {
      std::vector<unsigned char> src_data = msg->data;
      cv::Mat bayer(msg->height, msg->width, CV_8UC1);
      uchar *data = bayer.data;
      for(size_t i = 0; i < msg->height * msg->width * 2; i += 2)
          *(data++) = ( 255 * (src_data[i] << 8 | src_data[i+1]) ) / 65536;
      dst = cv::Mat(msg->height, msg->width, CV_8UC3);
      cv::cvtColor(bayer, dst, CV_BayerRG2RGB);
  }
  else if(encoding == "rgb8") // RGB24, YUV444, YUV422, YUV411
  {
      dst = cv::Mat(msg->height, msg->width, CV_8UC3);
      copyVectorToMat(msg->data, dst);
      copyVectorToMat(msg->data, dst);
  }
  else if(encoding == "bgr8") // raw8
  {
      cv::Mat bayer(msg->height, msg->width, CV_8UC3);
      copyVectorToMat(msg->data, bayer);
      dst = cv::Mat(msg->height, msg->width, CV_8UC3);
      cv::cvtColor(bayer, dst, CV_BGR2RGB);
  }
  else
  {
      ROS_INFO("Unrecognized encoding: [%s]", encoding.c_str());
  }

  float aspect_ratio = (float)dst.cols / (float)dst.rows;

  cv::resize(dst, dst, cv::Size(aspect_ratio*resizeHeight,resizeHeight));

  cv_bridge::CvImage cvi;
  cvi.header.stamp = msg->header.stamp;
  cvi.header.frame_id = "image";
  cvi.encoding = ( dst.channels() > 1 ? "rgb8" : "mono8" );
  cvi.image = dst;

  sensor_msgs::ImageConstPtr im = cvi.toImageMsg();
  pub.publish(im);
}

void ImageRepublisher::copyVectorToMat(const std::vector<uint8_t> &v, cv::Mat &m)
{
    auto data = m.data;
    for(auto element : v)
        *(data++) = element;
}

}
