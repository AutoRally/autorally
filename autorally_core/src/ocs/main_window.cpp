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
 * @file main_window.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date March 2, 2012
 * @copyright 2013 Georgia Institute of Technology
 * @brief Implementation for the qt-based ocs gui.
 **/
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "main_window.hpp"
#include <QDir>

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    m_savingImages = false;
    m_saveOneImage = 0;

  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	ui.setupUi(this);
	// qApp is a global variable for the application
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

  ReadSettings();
	setWindowIcon(QIcon(":/doc/car_logo.png"));

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	ui.diagMsgsTreeView->setModel(qnode.diagnosticModel());
	ui.diagMsgsTreeView->header()->setResizeMode(QHeaderView::ResizeToContents);

	ui.safeSpeedTreeView->setModel(qnode.safeSpeedModel());
  ui.safeSpeedTreeView->header()->setResizeMode(QHeaderView::ResizeToContents);

  ui.imageMaskTreeView->setModel(qnode.imageMaskModel());
  ui.imageMaskTreeView->header()->setResizeMode(QHeaderView::ResizeToContents);


  QObject::connect(ui.motionControlButton, SIGNAL(clicked(bool)),
                   this, SLOT(enableMotion(bool)));
  QObject::connect(ui.safeSpeedSetButton, SIGNAL(clicked(bool)),
                   this, SLOT(setSafeSpeed(bool)));
//  QObject::connect(ui.steeringControlSlider, SIGNAL(valueChanged(int)),
//                   this, SLOT(setSteering(int)));
//  QObject::connect(ui.throttleControlSlider, SIGNAL(valueChanged(int)),
//                   this, SLOT(setThrottle(int)));
//  QObject::connect(ui.frontBrakeControlSlider, SIGNAL(valueChanged(int)),
//                   this, SLOT(setFrontBrake(int)));
//  QObject::connect(ui.backBrakeControlSlider, SIGNAL(valueChanged(int)),
//                   this, SLOT(setBackBrake(int)));
  QObject::connect(ui.tab_manager, SIGNAL(currentChanged(int)),
                   this, SLOT(currentTabChanged(int)));

  QObject::connect(&qnode, SIGNAL(newWheelSpeeds(
                      const autorally_msgs::wheelSpeedsConstPtr&)),
                   this, SLOT(updateWheelSpeeds(
                      const autorally_msgs::wheelSpeedsConstPtr&)));
  //QObject::connect(&qnode, SIGNAL(newArduinoData(
  //                    const autorally_msgs::arduinoDataConstPtr&)),
  //                 this, SLOT(updateArduinoData(
  //                    const autorally_msgs::arduinoDataConstPtr&)));
  QObject::connect(&qnode, SIGNAL(newServoData(
                 const autorally_msgs::servoMSGConstPtr&)),
                 this, SLOT(updateServoData(
                 const autorally_msgs::servoMSGConstPtr&)));
  QObject::connect(&qnode, SIGNAL(newImage1()),
                   this, SLOT(updateImage1()));
  QObject::connect(&qnode, SIGNAL(newImage2()),
                   this, SLOT(updateImage2()));

  QObject::connect(ui.controlButton, SIGNAL(clicked(const bool)),
                   this, SLOT(setControl(const bool)));
  QObject::connect(ui.clearStaleDiagButton, SIGNAL(released()),
                   &qnode.m_diagModel, SLOT(clearStaleDiag()));
  QObject::connect(ui.diagMsgsTreeView,
                   SIGNAL(doubleClicked(const QModelIndex&)),
                   &qnode.m_diagModel, SLOT(diagModelDoubleClicked(const QModelIndex&)));
  QObject::connect(ui.safeSpeedTreeView,
                   SIGNAL(doubleClicked(const QModelIndex&)),
                   &qnode, SLOT(safeSpeedModelDoubleClicked(const QModelIndex&)));

  QObject::connect(&m_updateTimeBoxesTimer, SIGNAL(timeout()),
                   this, SLOT(updateTimeBoxes()));
  QObject::connect(&m_diagTimeTimer, SIGNAL(timeout()),
                   &qnode, SLOT(updateTimes()));
  QObject::connect(&m_diagTimeTimer, SIGNAL(timeout()),
                   &qnode.m_diagModel, SLOT(updateTimes()));
  QObject::connect(&m_servoCommandTimer, SIGNAL(timeout()),
                   this, SLOT(sendServoCommand()));

  m_diagTimeTimer.start(75);
  m_updateTimeBoxesTimer.start(100);
//  m_servoCommandTimer.start(100);

  ui.motionControlButton->setStyleSheet("QPushButton:checked{background-color: \
    red;} QPushButton:!checked{background-color: green;}");

  m_progrssBarLevelStyleSheets[0] = "QProgressBar { \
                                    text-align: center;\
                                    } \
                                    QProgressBar::chunk { \
                                       background-color: green;\
                                       border-radius: 5px; }";
  m_progrssBarLevelStyleSheets[1] = "QProgressBar { \
                                    text-align: center;\
                                    } \
                                    QProgressBar::chunk { \
                                       background-color: yellow;\
                                       border-radius: 5px; }";
  m_progrssBarLevelStyleSheets[2] = "QProgressBar { \
                                    text-align: center;\
                                    } \
                                    QProgressBar::chunk { \
                                       background-color: red;\
                                       border-radius: 5px; }";

  ui.throttleBar->setFormat("Throttle: %v");
  ui.backBrakeBar->setFormat("Back Brake: %v");
  ui.frontBrakeBar->setFormat("Front Brake: %v");
  m_servoCommand.header.frame_id = "OCS";
  m_servoCommand.steering = -5.0;
  m_servoCommand.throttle = -5.0;
  m_servoCommand.frontBrake = -5.0;
  m_servoCommand.backBrake = -5.0;
  //automatically connect into ROS system on startup
  qnode.init();
  m_startTime = ros::Time::now();

  ui.tab_manager->setCurrentIndex(0);
  currentTabChanged(0);
  
  ui.imageTopics_comboBox->setEditable(false);
  on_imageTopicsRefresh_button_clicked();
  ui.saveImagesPath_lineEdit->setText(tr("/media/data/").append(QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss")));
}

MainWindow::~MainWindow() {}

void MainWindow::updateWheelSpeeds(const autorally_msgs::wheelSpeedsConstPtr& msg)
{
  QString text;
  text.sprintf("%.2f", (msg->lfSpeed+msg->rfSpeed)/2.0);
  ui.speedLabel->setText(text);
  
  text.sprintf("%.2f", msg->lfSpeed);
  ui.wheelRPS_fl->setText(text);
  text.sprintf("%.2f", msg->rfSpeed);
  ui.wheelRPS_fr->setText(text);
  text.sprintf("%.2f", msg->lbSpeed);
  ui.wheelRPS_bl->setText(text);
  text.sprintf("%.2f", msg->rbSpeed);
  ui.wheelRPS_br->setText(text);

}

/*
void MainWindow::updateArduinoData(
                 const autorally_msgs::arduinoDataConstPtr& msg)
{
  QString text;
  text.sprintf("%.2f", msg->leftFrontHallEffect);
  ui.wheelRPS_fl->setText(text);
  text.sprintf("%.2f", msg->rightFrontHallEffect);
  ui.wheelRPS_fr->setText(text);
  text.sprintf("%.2f", msg->leftBackHallEffect);
  ui.wheelRPS_bl->setText(text);
  text.sprintf("%.2f", msg->rightBackHallEffect);
  ui.wheelRPS_br->setText(text);
}
*/

void MainWindow::updateServoData(const autorally_msgs::servoMSGConstPtr& msg)
{
//  {
//  if(msg->throttle >= 0)
//    ui.throttleBar->setValue(100*msg->throttle);
//    ui.backBrakeBar->setValue(0);
//  } else
//  {
//    ui.throttleBar->setValue(0);
//    ui.backBrakeBar->setValue(-msg->throttle);
//  }

  ui.steeringSlider->setValue(100*msg->steering);
  ui.throttleBar->setValue(100*msg->throttle);
  ui.frontBrakeBar->setValue(100*msg->frontBrake);
  ui.backBrakeBar->setValue(100*msg->backBrake);
  ui.safeSpeedLabel->setNum(msg->safeSpeed);
}

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>Auto-Rally OCS 0.10</h2><p>Copyright (2013) C Georgia Institute of Technology</p><p>Remote Operator Control Station for communicate with Auto-Rally vehicles</p>"));
}

void MainWindow::on_imageTopicsRefresh_button_clicked() {
    std::vector<std::string> topics;
    qnode.getImageTopics(topics);
    ui.imageTopics_comboBox->clear();
    ui.imageTopics_comboBox_2->clear();
    ui.imageTopics_comboBox->addItem("None");
    ui.imageTopics_comboBox_2->addItem("None");
    for(size_t i = 0; i < topics.size(); i++)
    {
        ui.imageTopics_comboBox->addItem(topics[i].c_str());
        ui.imageTopics_comboBox_2->addItem(topics[i].c_str());
    }
}

void MainWindow::on_pushButton_saveLeft_clicked() {
    m_saveOneImage = 1;
}

void MainWindow::on_pushButton_saveRight_clicked() {
    m_saveOneImage = 2;
}

void MainWindow::on_saveImages_button_clicked() {
    ui.saveImagesPath_lineEdit->setEnabled(!ui.saveImages_button->isChecked());
    m_savingImages = ui.saveImages_button->isChecked();
}

void MainWindow::on_imageTopics_comboBox_currentIndexChanged(int index)
{
    qnode.switchImageTopic(0, ui.imageTopics_comboBox->itemText(index).toStdString());
}

void MainWindow::on_imageTopics_comboBox_2_currentIndexChanged(int index)
{
    qnode.switchImageTopic(1, ui.imageTopics_comboBox_2->itemText(index).toStdString());
}

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "ocs");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "ocs");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::enableMotion(const bool check)
{
  if(check)
  {
    //ui.safeSpeedValueBox->setEnabled(true);
    ui.safeSpeedSetButton->setEnabled(true);
    ui.motionControlButton->setText("Enable Motion");
    qnode.setSafeSpeed(0.0);
  } else
  {
    //ui.safeSpeedValueBox->setEnabled(false);
    ui.safeSpeedSetButton->setEnabled(false);
    ui.motionControlButton->setText("STOP");
    setSafeSpeed(ui.safeSpeedSetButton->isChecked());
  }
}

void MainWindow::setSafeSpeed(const bool check)
{
  if(check)
  {
    ui.safeSpeedValueBox->setEnabled(false);
    if(!ui.motionControlButton->isChecked())
    {
      qnode.setSafeSpeed(ui.safeSpeedValueBox->value());
    }
  } else
  {
    ui.safeSpeedValueBox->setEnabled(true);
    if(!ui.motionControlButton->isChecked())
    {
      qnode.setSafeSpeed(25.0);
    }
  }
}

void MainWindow::setControl(bool check)
{
  if(check)
  {
    m_servoCommandTimer.start(100);
  } else
  {
    m_servoCommandTimer.stop();
  }
  //m_sendServoCommand = check;
}

void MainWindow::sendServoCommand()
{
//  if(ui.steeringControlEnable->checkState() == Qt::Checked)
//  {
//    m_servoCommand.steering = val/100.0;
//    qnode.servoControl(m_servoCommand);
//  } else
//  {
//    m_servoCommand.steering = -5.0;
//  }

  if(ui.steeringControlEnable->checkState() == Qt::Checked)
  {
    m_servoCommand.steering = ui.steeringControlSlider->value()/100.0;
    qnode.servoControl(m_servoCommand);
  } else
  {
    m_servoCommand.steering = -5.0;
  }

  if(ui.throttleControlEnable->checkState() == Qt::Checked)
  {
    m_servoCommand.throttle = ui.throttleControlSlider->value()/100.0;
    qnode.servoControl(m_servoCommand);
  } else
  {
    m_servoCommand.throttle = -5.0;
  }

  if(ui.frontBrakeControlEnable->checkState() == Qt::Checked)
  {
    m_servoCommand.frontBrake = ui.frontBrakeControlSlider->value()/100.0;
    qnode.servoControl(m_servoCommand);
  } else
  {
    m_servoCommand.frontBrake = -5.0;
  }

  if(ui.backBrakeControlEnable->checkState() == Qt::Checked)
  {
    m_servoCommand.backBrake = ui.backBrakeControlSlider->value()/100.0;
    qnode.servoControl(m_servoCommand);
  } else
  {
    m_servoCommand.backBrake = -5.0;
  }

  qnode.servoControl(m_servoCommand);
}

void MainWindow::setSteering(const int val)
{
  if(ui.steeringControlEnable->checkState() == Qt::Checked)
  {
    m_servoCommand.steering = val/100.0;
    qnode.servoControl(m_servoCommand);
  } else
  {
    m_servoCommand.steering = -5.0;
  }
}

void MainWindow::setThrottle(const int val)
{
  if(ui.throttleControlEnable->checkState() == Qt::Checked)
  {
    m_servoCommand.throttle = val/100.0;
    qnode.servoControl(m_servoCommand);
  } else
  {
    m_servoCommand.throttle = -5.0;
  }
}

void MainWindow::setFrontBrake(const int val)
{
  if(ui.frontBrakeControlEnable->checkState() == Qt::Checked)
  {
    m_servoCommand.frontBrake = val/100.0;
    qnode.servoControl(m_servoCommand);
  } else
  {
    m_servoCommand.frontBrake = -5.0;
  }
}

void MainWindow::setBackBrake(const int val)
{
  if(ui.backBrakeControlEnable->checkState() == Qt::Checked)
  {
    m_servoCommand.backBrake = val/100.0;
    qnode.servoControl(m_servoCommand);
  } else
  {
    m_servoCommand.backBrake = -5.0;
  }
}

void MainWindow::updateTimeBoxes()
{
  ros::Time time = ros::Time::now();
  ui.timeSpinBox->setValue(time.toSec());
  ui.elapsedSpinBox->setValue( (time-m_startTime).toSec());
  //ROS_INFO("%f",(time-m_startTime).toSec());
}

void MainWindow::currentTabChanged(const int /*index*/)
{
  qnode.m_currentTabText =
            ui.tab_manager->tabText(ui.tab_manager->currentIndex()).toStdString();
}

void MainWindow::updateImage1()
{
  int lock_ret = pthread_mutex_lock(&qnode.m_imageMutex);
  if(lock_ret != 0) {
    switch(lock_ret) {
      case EDEADLK:
        ROS_WARN("A deadlock was detected on the image mutex. Skipping this frame.");
        break;
      case EAGAIN:
        ROS_WARN("The image mutex received too many recursive locks. Skipping this frame.");
        break;
      case ENOTRECOVERABLE:
        ROS_WARN("The state of the image mutex is not recoverable. Skipping this frame.");
        break;
      default:
        ROS_WARN("Unknown error while locking image mutex. Skipping this frame.");
    }
    return;
  }
  QPainter painter(&qnode.m_firewireImage1);

  std::map<std::string, autorally_msgs::imageMask>::const_iterator mapIt;
  for(mapIt = qnode.m_imMaskModel.m_masks.begin();
      mapIt != qnode.m_imMaskModel.m_masks.end();
      mapIt++)
  {
    const QStandardItem* item = qnode.m_imMaskModel.itemFromName(mapIt->second.sender);
    if(item)
    {
      if(item->checkState() == Qt::Checked)
      {
        QPen pen(item->background().color());
        pen.setWidth(4);
        painter.setPen(pen);

        std::vector<autorally_msgs::point2D>::const_iterator pointIt;
        for(pointIt = mapIt->second.points.begin();
           pointIt != mapIt->second.points.end();
           pointIt++)
        {
          painter.drawPoint(pointIt->x, pointIt->y);
        }

        std::vector<autorally_msgs::line2D>::const_iterator vecIt;
        for(vecIt = mapIt->second.lines.begin();
           vecIt != mapIt->second.lines.end();
           vecIt++)
        {
          painter.drawLine(vecIt->start.x, vecIt->start.y, vecIt->end.x, vecIt->end.y);
        }

        std::vector<sensor_msgs::RegionOfInterest>::const_iterator rectIt;
        for(rectIt = mapIt->second.rois.begin();
           rectIt != mapIt->second.rois.end();
           rectIt++)
        {
          painter.drawRect(rectIt->x_offset, rectIt->y_offset, rectIt->width, rectIt->height);
        }
      }
    }
  }

  painter.end();
  //set image
  if(qnode.m_currentTabText == "Video")
  {
    ui.cameraLabel->setPixmap(qnode.m_firewireImage1.scaled(240,240, Qt::KeepAspectRatio));
  }
  
  if(m_savingImages || m_saveOneImage == 1)
  {
    QDir().mkpath(ui.saveImagesPath_lineEdit->text().append(ui.imageTopics_comboBox->currentText()));
    QString path = ui.saveImagesPath_lineEdit->text().append(ui.imageTopics_comboBox->currentText()).append(QDateTime::currentDateTime().toString("/yyyy-MM-dd-hh-mm-ss-zzz")).append(".png");
    if(!qnode.m_firewireImage1.save(path))
    {
        std::cout << "Failed to save image to " << path.toStdString().c_str() << std::endl;
    }
    m_saveOneImage = 0;
  }
  
  pthread_mutex_unlock(&qnode.m_imageMutex);
}

void MainWindow::updateImage2()
{
  int lock_ret = pthread_mutex_lock(&qnode.m_imageMutex);
  if(lock_ret != 0) {
    switch(lock_ret) {
      case EDEADLK:
        ROS_WARN("A deadlock was detected on the image mutex. Skipping this frame.");
        break;
      case EAGAIN:
        ROS_WARN("The image mutex received too many recursive locks. Skipping this frame.");
        break;
      case ENOTRECOVERABLE:
        ROS_WARN("The state of the image mutex is not recoverable. Skipping this frame.");
        break;
      default:
        ROS_WARN("Unknown error while locking image mutex. Skipping this frame.");
    }
    return;
  }
  QPainter painter(&qnode.m_firewireImage2);

  std::map<std::string, autorally_msgs::imageMask>::const_iterator mapIt;
  for(mapIt = qnode.m_imMaskModel.m_masks.begin();
      mapIt != qnode.m_imMaskModel.m_masks.end();
      mapIt++)
  {
    const QStandardItem* item = qnode.m_imMaskModel.itemFromName(mapIt->second.sender);
    if(item)
    {
      if(item->checkState() == Qt::Checked)
      {
        QPen pen(item->background().color());
        pen.setWidth(4);
        painter.setPen(pen);

        std::vector<autorally_msgs::point2D>::const_iterator pointIt;
        for(pointIt = mapIt->second.points.begin();
           pointIt != mapIt->second.points.end();
           pointIt++)
        {
          painter.drawPoint(pointIt->x, pointIt->y);
        }

        std::vector<autorally_msgs::line2D>::const_iterator vecIt;
        for(vecIt = mapIt->second.lines.begin();
           vecIt != mapIt->second.lines.end();
           vecIt++)
        {
          painter.drawLine(vecIt->start.x, vecIt->start.y, vecIt->end.x, vecIt->end.y);
        }

        std::vector<sensor_msgs::RegionOfInterest>::const_iterator rectIt;
        for(rectIt = mapIt->second.rois.begin();
           rectIt != mapIt->second.rois.end();
           rectIt++)
        {
          painter.drawRect(rectIt->x_offset, rectIt->y_offset, rectIt->width, rectIt->height);
        }
      }
    }
  }

  painter.end();
  //set image
  if(qnode.m_currentTabText == "Video")
  {
    ui.cameraLabel_2->setPixmap(qnode.m_firewireImage2.scaled(240,240, Qt::KeepAspectRatio));
  }
  
  if(m_savingImages || m_saveOneImage == 2)
  {
    QDir().mkpath(ui.saveImagesPath_lineEdit->text().append(ui.imageTopics_comboBox->currentText()));
    QString path = ui.saveImagesPath_lineEdit->text().append(ui.imageTopics_comboBox->currentText()).append(QDateTime::currentDateTime().toString("/yyyy-MM-dd-hh-mm-ss-zzz")).append(".png");
    if(!qnode.m_firewireImage2.save(path))
    {
        std::cout << "Failed to save image to " << path.toStdString().c_str() << std::endl;
    }
    m_saveOneImage = 0;
  }
  
  pthread_mutex_unlock(&qnode.m_imageMutex);
}

template <class T>
unsigned char MainWindow::levelFromParams(const T warnVal, const T critVal, const T val) const
{
  unsigned char level = 0;
  //std::cout << warnVal << " " << critVal << " " << val << std::endl;
  if(critVal > warnVal)
  {
    if(val >= warnVal) ++level;
    if(val >= critVal) ++level;
  }
  else if (warnVal > critVal)
  {
    if(val <= warnVal) ++level;
    if(val <= critVal) ++level;
  }

  return level;
}

