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
/**
 * @file qnode.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date March 2, 2012
 * @brief Implementation of ROS components for ocs gui.
 **/
#include <std_msgs/String.h>
#include <sstream>
#include "qnode.hpp"
#include <ros/master.h>
#include <iostream>

QNode::QNode(int argc, char** argv ) :
  m_currentTabText(""),
	m_firewireImage1(320,320),
	m_firewireImage2(320,320),
  init_argc(argc),
  init_argv(argv)
{
  //register my data types so I can pass them around in signals and slots
	qRegisterMetaType<autorally_msgs::runstopConstPtr>
	                ("autorally_msgs::runstopConstPtr");
	qRegisterMetaType<autorally_msgs::wheelSpeedsConstPtr>
	                ("autorally_msgs::wheelSpeedsConstPtr");
  //qRegisterMetaType<autorally_msgs::servoMSGConstPtr>
	//                ("autorally_msgs::servoMSGConstPtr");
  qRegisterMetaType<autorally_msgs::chassisStateConstPtr>
                  ("autorally_msgs::chassisStateConstPtr");
  qRegisterMetaType<autorally_msgs::chassisCommandConstPtr>
                  ("autorally_msgs::chassisCommandConstPtr");
  qRegisterMetaType<diagnostic_msgs::DiagnosticStatus>
	                ("diagnostic_msgs::DiagnosticStatus");
  

  int init_err = pthread_mutex_init(&m_imageMutex, nullptr);
  if(init_err != 0) {
    switch(init_err) {
      case EAGAIN:
        std::cerr << "ERROR: Unable to initialize image mutex due to lack of resource other than memory." << std::endl;
        break;
      case ENOMEM:
        std::cerr << "ERROR: Unable to initialize image mutex due to lack of memory." << std::endl;
        break;
      case EPERM:
        std::cerr << "ERROR: Unable to initialize image mutex due to insufficient priveleges." << std::endl;
        break;
      default:
        std::cerr << "ERROR: Unable to initialize image mutex for unknown reason." << std::endl;
        break;
    }
    exit(init_err);
  }
}

QNode::~QNode() {
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"ocs");
	if ( ! ros::master::check() ) {
		return false;
	}
	
	m_nh = new ros::NodeHandle;

	// Add your ros communications here.
	runstop_publisher = m_nh->advertise<autorally_msgs::runstop>
	                      ("runstop", 1);
	m_chassisCommandPub = m_nh->advertise<autorally_msgs::chassisCommand>
                        ("OCS/chassisCommand", 1);

  runstop_subscriber = m_nh->subscribe("runstop", 5,
                                      &QNode::runstopCallback,
                                      this);
  wheelSpeeds_subscriber = m_nh->subscribe("wheelSpeeds", 1,
                                      &QNode::wheelSpeedsCallback,
                                      this);
	diagStatus_subscriber = m_nh->subscribe("diagnostics", 10,
                                      &QNode::diagStatusCallback,
                                      this);
	chassisState_subscriber = m_nh->subscribe("chassisState", 1,
                                      &QNode::chassisStateCb,
                                      this);
  imageMask_subscriber = m_nh->subscribe("imageMask", 10,
                                      &QNode::imageMaskCallback,
                                      this);

	m_runstopTimer = m_nh->createTimer(ros::Duration(0.1), &QNode::ssTimerCallback, this);
//	m_servoCommandTimer = n.createTimer(ros::Duration(0.1), &QNode::ssTimerCallback, this);
//	m_servoCommandTimer.stop();

	m_runstop.sender = "OCS";

  QStringList header;
  header << "Sender" << "Motion Enabled" << "Time since last message";
	m_runstopModel.setColumnCount(3);
  m_runstopModel.setHorizontalHeaderLabels(header);

  double diagFreq;
  ros::param::param<double>("diagnosticsFrequency", diagFreq, 1.0);
  ros::param::param<double>("runstop/Timeout", m_runstopTimeout, 5.0);

  m_diagModel.setDiagnosticFrequency(diagFreq);

	start();
	return true;
}

void QNode::ssTimerCallback(const ros::TimerEvent&)
{
  m_runstop.header.stamp = ros::Time::now();
  runstop_publisher.publish(m_runstop);
}

void QNode::run() {
	ros::spin();

	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	emit rosShutdown(); //used to signal the gui to shutdown (useful to roslaunch)
}

void QNode::runstopCallback(const autorally_msgs::runstopConstPtr& msg)
{
  QList<QStandardItem *> items = m_runstopModel.findItems(
                                          QString::fromStdString(msg->sender) );
  if(items.isEmpty())
  {
    //add new item for the sender
    m_runstopModel.appendRow(generateNewrunstop(msg));
    items = m_runstopModel.findItems( QString::fromStdString(msg->sender) );
  }

  if(items.size() == 1)
  {
    //update the runstop
    m_runstopModel.item(items.front()->index().row(),1)->
        setText(QString::number(msg->motionEnabled, 'g', 4));
    m_runstopModel.item(items.front()->index().row(),2)->
       setData(QVariant(msg->header.stamp.toSec()));
  } else //found more than one sender with the same name
  {
    ROS_ERROR("Something is wrong with the OCS runstopModel!!!!!");
  }
}

void QNode::runstopModelDoubleClicked(const QModelIndex& index)
{
  m_runstopModel.removeRows(index.row(), 1);
}

void QNode::wheelSpeedsCallback(const autorally_msgs::wheelSpeedsConstPtr& msg)
{
  emit newWheelSpeeds(msg);
}

void QNode::chassisStateCb(const autorally_msgs::chassisStateConstPtr& msg)
{
  emit newChassisState(msg);
}

void QNode::setRunstop(const double& speed)
{
  if(speed > 0)
  {
    m_runstop.motionEnabled = true;
  } else
  {
    m_runstop.motionEnabled = false;
  }
}

void QNode::diagStatusCallback(const diagnostic_msgs::DiagnosticArray& msg)
{
  m_diagModel.update(msg);
}


void QNode::actuatorControl(autorally_msgs::chassisCommand& msg)
{
//  if(m_sendServoCommand)
  {
    msg.header.stamp = ros::Time::now();
    m_chassisCommandPub.publish(msg);
  }
}

QList<QStandardItem*> QNode::generateNewrunstop(
                     const autorally_msgs::runstopConstPtr& msg)
{
    QList<QStandardItem*> newrunstop;
    newrunstop << new QStandardItem(msg->sender.c_str());
    newrunstop << new QStandardItem(QString::number(msg->motionEnabled, 'g', 4));
    newrunstop << new QStandardItem("0.000");

    newrunstop[0]->setEditable(false);
    newrunstop[1]->setEditable(false);
    newrunstop[2]->setEditable(false);
    newrunstop[2]->setData(QVariant(msg->header.stamp.toSec()));

    return newrunstop;
}

void QNode::updateTimes()
{
  double time;
  QStandardItem* node;

  if(m_currentTabText == "System Info")
  {
    for(int i = 0; i < m_runstopModel.rowCount(); i++)
    {
      if( (node = m_runstopModel.item(i,2)) == 0 || !node->data().isValid())
      {
        ROS_ERROR("Invalid runstop child");
      } else
      {
        time = ros::Time::now().toSec()-node->data().toDouble();
        node->setText(QString::number(time, 'g', 4));
        //set colors for stale
        if(time > m_runstopTimeout)
        {
          node->setBackground(Qt::magenta);
        } else
        {
          node->setBackground(m_runstopModel.item(i,1)->background());
        }
      }
    }
  }
}

void QNode::imageMaskCallback(const autorally_msgs::imageMask& msg)
{
  m_imMaskModel.update(msg);
}

void QNode::imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
  QImage img = formatImageForDisplay(msg);

  if(!pthread_mutex_trylock(&m_imageMutex))
  {
    if(m_firewireImage1.convertFromImage(img))
    {
      emit newImage1();
    }
    pthread_mutex_unlock(&m_imageMutex);
  }
}

void QNode::imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
  QImage img = formatImageForDisplay(msg);

  if(!pthread_mutex_trylock(&m_imageMutex))
  {
    if(m_firewireImage2.convertFromImage(img))
    {
      emit newImage2();
    }
    pthread_mutex_unlock(&m_imageMutex);
  }
}

unsigned char* QNode::convertBGRtoRGB(const unsigned char* data, int size) {
  unsigned char* newData = new unsigned char[size];
  for(int i = 0; i < size; i+=3){
    // Swap R & B channels
    char tmp = data[i];
    newData[i] = data[i+2];
    newData[i+2] = tmp;
    // Copy G channel directly
    newData[i+1] =  data[i+1];
  }
  return newData;
}

void QNode::getImageTopics(std::vector<std::string>& topics)
{
  /* 
   * Largely copied from rqt_image_view/image_view.cpp
   */
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(*m_nh);
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    //qDebug("ImageView::updateTopicList() declared transport '%s'", it->c_str());
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
    {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (it->datatype == "sensor_msgs/Image")
    {
      QString topic = it->name.c_str();

      topics.push_back(topic.toStdString());

      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.push_back(sub.toStdString());
        }
      }
    }
    if (it->datatype == "sensor_msgs/CompressedImage")
    {
      QString topic = it->name.c_str();
      int index = topic.lastIndexOf("/");
      if (index != -1)
      {
        topic.replace(index, 1, " ");
        topics.push_back(topic.toStdString());
      }
    }
  }
}

void QNode::switchImageTopic(int subscriber, std::string name)
{
    image_transport::Subscriber& sub = subscriber ? images_subscriber_2 : images_subscriber;
    void (QNode::*callback)(const sensor_msgs::ImageConstPtr& msg) = subscriber ? &QNode::imageCallback2 : &QNode::imageCallback1;
    if(sub.getTopic() != name)
    {
        sub.shutdown();
        if(name != "None")
        {
            image_transport::ImageTransport it(*m_nh);
            if(find(name.begin(), name.end(), ' ') == name.end())
            {
                sub = it.subscribe(name, 1, callback,this);
            }
            else
            {
                std::string base = name.substr(0,name.find(' '));
                std::string param = name.substr(name.find(' ')+1, name.length()-name.find(' '));
                sub = it.subscribe(base, 1,callback,this,param);
            }
        }
    }
}

QImage QNode::formatImageForDisplay(const sensor_msgs::ImageConstPtr &msg)
{
  QImage::Format format = QImage::Format_Indexed8;

  std::string encoding = msg->encoding;

  unsigned char *newData = nullptr;

  if(encoding == "bgr8") {
    format = QImage::Format_RGB888;
    newData = convertBGRtoRGB(&msg->data[0], msg->step * msg->height);
  }
  else if(encoding == "rgb8") {
    format = QImage::Format_RGB888;
  }
  else {
    ROS_WARN_STREAM("The OCS does not currently support " << encoding << " images. This image topic will not render correctly.");
  }

  QImage img = QImage(encoding == "bgr8" ? newData : &msg->data[0], msg->width, msg->height, msg->step, format).copy();

  delete newData;

  return img;
}