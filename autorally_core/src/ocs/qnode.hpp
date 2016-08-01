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
 * @file qnode.hpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date March 3, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief ROS backend for OCS
 *
 * @details This file contains the QNode class that implements the ROS backend
 * for the operator control station passing messages from and to the rest of the
 * nodes in the system.
 ***********************************************/
#ifndef QNODE_HPP_
#define QNODE_HPP_

#include <pthread.h>
#include <string>
#include <vector>

#include <QThread>
#include <QStringListModel>
#include <QtGui/QStandardItemModel>
#include <QtGui/QStandardItem>
#include <QtGui/QPushButton>
#include <QtGui/QTableWidgetItem>
#include <QtGui/QPixmap>

#ifndef Q_MOC_RUN
  #include <ros/ros.h>
  #include <ros/time.h>
  #include <image_transport/image_transport.h>
  #include <diagnostic_msgs/DiagnosticArray.h>
  #include <autorally_msgs/chassisState.h>
  #include <autorally_msgs/chassisCommand.h>
  #include <autorally_msgs/wheelSpeeds.h>
  #include <autorally_msgs/runstop.h>
  #include "ImageMaskEntry.hpp"
  #include "DiagnosticsEntry.hpp"
#endif

/**
 *  @class QNode qnode.hpp "ocs/qnode.hpp"
 *  @brief ROS backend for OCS
 *
 *  QNode is ROS the backend for the Operator Control Station (OCS) that
 *  receives and sends all pertinent ROS messages. The class also handles
 *  updating the models used by the tree views in the OCS to display data.
 *  Updating many of the individual text boxes is passed back to OCS through
 *  signals.
 *  @note this code grows orgnaically (its dirty)
 */
class QNode : public QThread {
  Q_OBJECT
public:

  std::string m_currentTabText;
  QPixmap m_firewireImage1;
  QPixmap m_firewireImage2;

  DiagnosticsEntry m_diagModel;
  ImageMaskEntry m_imMaskModel;
  pthread_mutex_t m_imageMutex;

  /**
  * @brief Constructor, just initializes internal variables
  *
  */
  QNode(int argc, char** argv );
  virtual ~QNode();

	/**
  * @brief Publishes and subscribes to all necessary ROS information, sets up
  *        view headers.
  * @note init() muse be called prior to run() for proper setup
  */
	bool init();

	/**
  * @brief Main loop for program
  *
  * Call run() after setup is complete to start listening/publishing ROS
  * messages. This function will not return until the ROS system is shut down.
  */
  void run();

	//QStringListModel* loggingModel() { return &logging_model; }
  /**
  * @brief Accessor for the QStandardItemModel holding diagnostic messages
  * @return QStandardItemModel* Pointer to the model
  */
  QStandardItemModel* diagnosticModel() { return m_diagModel.model(); }

	/**
  * @brief Accessor for the QStandardItemModel holding runstop messages
  * @return QStandardItemModel* Pointer to the model
  */
  QStandardItemModel* runstopModel() { return &m_runstopModel; }

  /**
  * @brief Accessor for the QStandardItemModel holding image masks
  * @return QStandardItemModel* Pointer to the model
  */
  QStandardItemModel* imageMaskModel() { return m_imMaskModel.model(); }

  /**
  * @brief Callback for new vehicleSpeed messages
  * @param msg The new vehicle speed
  */
  void wheelSpeedsCallback(const autorally_msgs::wheelSpeedsConstPtr& msg);

  /**
  * @brief Callback for new runstop messages
  * @param msg The new vehicle speed
  */
  void runstopCallback(const autorally_msgs::runstopConstPtr& msg);

  /**
  * @brief Callback for new wheelSpeeds messages
  * @param msg The new wheelSpeeds
  */
  //void wheelSpeedsCallback(const autorally_msgs::wheelSpeedsConstPtr& msg);

  /**
  * @brief Callback for new chassisState messages
  * @param msg The new chassisState message
  */
  void chassisStateCb(const autorally_msgs::chassisStateConstPtr& msg);

  /**
  * @brief Callback for new diagnostics messages
  * @param msg the new diagnostic message
  */
  void diagStatusCallback(const diagnostic_msgs::DiagnosticArray& msg);

  /**
  * @brief Callback for new imageMask messages
  * @param msg The new imageMask
  */
  void imageMaskCallback(const autorally_msgs::imageMask& msg);

  /**
  * @brief Slot indicating the OCS runstop has been updated
  * @param speed new value
  */
  void setRunstop(const double& speed);

  /**
  * @brief Slot the desired chassisCommand message data to send
  * @param msg the control values from the OCS
  */
  void actuatorControl(autorally_msgs::chassisCommand& msg);
  
  void getImageTopics(std::vector<std::string>& topics);
  
  void switchImageTopic(int subscriber, std::string name);

signals:

  /**
  * @brief Signal that the ROS system has shut down
  */
  void rosShutdown();

  /**
  * @brief Signal for the OCS to update vehicleSpeed display
  * @param msg The new value
  */
  void newWheelSpeeds(const autorally_msgs::wheelSpeedsConstPtr& msg);

  /**
  * @brief Signal for the OCS to update data from the arduino
  * @param msg The new data
  */
  //void newWheelSpeeds(const autorally_msgs::wheelSpeedsConstPtr& msg);

  /**
  * @brief Signal for the OCS to update chassis information
  * @param msg The new data
  */
  void newChassisState(const autorally_msgs::chassisStateConstPtr& msg);

  void newImage1();
  void newImage2();

public slots:

  /**
  * @brief Update the time since last message received for all elements in OCS
  */
  void updateTimes();

  /**
  * @brief Callback for a user click on a runstop entry
  * @param index the index for the item clicked
  */
  void runstopModelDoubleClicked(const QModelIndex& index);

private:
	int init_argc; ///< Command line parameter count
	char** init_argv; ///< Command line parameters
	
	ros::NodeHandle *m_nh;

	ros::Timer m_runstopTimer; ///< Timer for sending runstop messages
	ros::Publisher runstop_publisher; ///< Publisher for runstop
	ros::Publisher m_chassisCommandPub; ///< Publisher for chassisCommand

	ros::Subscriber wheelSpeeds_subscriber; ///< Subscriber for wheelSpeeds
	ros::Subscriber runstop_subscriber; ///< Subscriber for runstop
	//ros::Subscriber wheelSpeeds_subscriber; ///< Subscriber for wheelSpeeds
	ros::Subscriber diagStatus_subscriber; ///< Subscriber for diagnostics
	ros::Subscriber chassisState_subscriber; ///< Subscriber for chassisState
	ros::Subscriber imageMask_subscriber; ///< Subscriber for imageMask
	image_transport::Subscriber images_subscriber; ///< Subscriber for images
	image_transport::Subscriber images_subscriber_2; ///< Subscriber for images

	autorally_msgs::runstop m_runstop; ///< runstop message OCS can send

  QStandardItemModel m_runstopModel; ///< Model holding runstop messages

  double m_runstopTimeout;

  void ssTimerCallback(const ros::TimerEvent&); ///< Callback to publish runstop
  void updateTimeBoxesCallback(const ros::TimerEvent&); ///< Callbak to update time displays

  /**
  * @brief Generates new entry for a safe speed message
  */
  QList<QStandardItem*> generateNewrunstop(
                    const autorally_msgs::runstopConstPtr& msg);

    /**
  * @brief Callback for new image messages
  * @param msg The new image data
  */
  void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
  void imageCallback2(const sensor_msgs::ImageConstPtr& msg);

  /**
  * @brief Swaps the first and third channel of the given image data.
  * @param data The original image data
  * @param size The size of the data buffer
  * @return A pointer to the new data. Don't forget to delete it when you're done.
  */
  unsigned char* convertBGRtoRGB(const unsigned char* data, int size);


  QImage formatImageForDisplay(const sensor_msgs::ImageConstPtr& msg);

};

#endif /* QNODE_HPP_ */
