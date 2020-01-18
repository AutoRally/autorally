#include "ImageRepublisher.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

PLUGINLIB_EXPORT_CLASS(autorally_core::ImageRepublisher, nodelet::Nodelet)

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

  
  image_transport::SubscriberStatusCallback subscriberCB = boost::bind(&ImageRepublisher::subscriberCallback, this);

  connectMutex.lock();

  pub = it.advertise("camera/image_display", 1, subscriberCB, subscriberCB);

  connectMutex.unlock();
  
}

void ImageRepublisher::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // Limit frame rate
  if( (ros::Time::now() - lastFrameTime).toSec() < secondsPerFrame )
  {
    return;
  }
  lastFrameTime = ros::Time::now();

  std::string encoding = msg->encoding;

  cv_bridge::CvImageConstPtr source = cv_bridge::toCvShare(msg, "rgb8");

  cv_bridge::CvImage resized;
  resized.header.stamp = msg->header.stamp;
  resized.header.frame_id = "image";
  resized.encoding = "rgb8";

  float aspect_ratio = (float)source->image.cols / (float)source->image.rows;

  cv::resize(source->image, resized.image, cv::Size(aspect_ratio*resizeHeight,resizeHeight));

  sensor_msgs::ImageConstPtr im = resized.toImageMsg();
  pub.publish(im);
}

void ImageRepublisher::copyVectorToMat(const std::vector<uint8_t> &v, cv::Mat &m)
{
    auto data = m.data;
    for(auto element : v)
        *(data++) = element;
}

void ImageRepublisher::subscriberCallback()
{
  connectMutex.lock();
  if(pub.getNumSubscribers() == 0)
  {
    sub.shutdown();
  } else if(!sub) {
    sub = it.subscribe("camera/image_raw", 1, &ImageRepublisher::imageCallback, this);
  }
  connectMutex.unlock();
}

}
