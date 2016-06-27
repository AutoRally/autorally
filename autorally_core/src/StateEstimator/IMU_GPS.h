
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
 * @file JumpControl.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date November 13, 2013
 * @copyright 2012 Georgia Institute of Technology
 * @brief JumpControl class definition
 *
 ***********************************************/

#ifndef IMU_GPS_H_
#define IMU_GPS_H_

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/base/timing.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/GPSFactor.h>

#include <list>
#include <iostream>
#include <fstream>
#include <queue>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "autorally_core/Diagnostics.h"
#include "BlockingQueue.h"

#include <autorally_msgs/servoMSG.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/imageMask.h>
#include <imu_3dm_gx4/FilterOutput.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265358979323846264338

using namespace gtsam;
using namespace GeographicLib;

namespace autorally_core
{
  class Imu_Gps : public Diagnostics
  {
  public:
    Imu_Gps();
    ~Imu_Gps();
    ros::NodeHandle m_nh;
    ros::Subscriber m_gpsSub, m_imuSub;
    ros::Publisher  m_posePub;
    ros::Publisher  m_biasAccPub, m_biasGyroPub;
//    ros::Publisher  m_anglePub, m_imuAnglePub;
    ros::Publisher  m_timePub;

    ros::Time m_prevTime;
    ros::Time m_lastImuTime;
    ros::Time m_overlimitTime;

    long m_biasKey, m_poseVelKey;

    double m_initialYaw;
    double m_lastImuT, m_lastImuTgps, m_imuQPrevTime;
    double m_initialRotationNoise, m_initialTransNoise, m_initialVelNoise;
    double m_initialBiasNoiseAcc, m_initialBiasNoiseGyro;
    double m_AccelBiasSigma, m_GyroBiasSigma;
    double m_gpsSigma, m_gravityMagnitude;
    double m_sensorX, m_sensorY, m_sensorZ;
    double m_sensorXAngle, m_sensorYAngle, m_sensorZAngle;
    double m_carXAngle, m_carYAngle, m_carZAngle;

    int m_gpsSkip, m_gpsCounter;
    int m_maxQSize;

    BlockingQueue<sensor_msgs::NavSatFixConstPtr> m_gpsOptQ;
    BlockingQueue<sensor_msgs::ImuConstPtr> m_ImuOptQ;
    boost::mutex m_optimizedStateMutex;
    NavState m_optimizedState;
    double m_optimizedTime;
    boost::shared_ptr<PreintegratedImuMeasurements> m_imuPredictor;
    double m_imuDt;
    imuBias::ConstantBias m_optimizedBias, m_previousBias;
    sensor_msgs::ImuConstPtr m_lastIMU;
    boost::shared_ptr<PreintegrationParams> m_preintegrationParams;

    std::list<sensor_msgs::ImuConstPtr> m_imuMeasurements, m_imuGrav;
    imu_3dm_gx4::FilterOutput m_initialPose;
    nav_msgs::Odometry m_initOdom;


    Vector3 m_gravity;
    Vector3 m_omegaCoriolis;
    Vector3 m_prevVel;
    Pose3 m_prevPose;
    Pose3 m_bodyPSensor, m_carENUPcarNED;

    LocalCartesian m_enu;   /// Object to put lat/lon coordinates into local cartesian
    bool m_gotFirstFix;
    bool m_invertx, m_inverty, m_invertz;

    SharedDiagonal priorNoisePose;
    SharedDiagonal priorNoiseVel;
    SharedDiagonal priorNoiseBias;

    Vector3 sigma_acc_bias_c;
    Vector3 sigma_gyro_bias_c;

    Vector noiseModelBetweenbias_sigma;
    SharedDiagonal noiseModelBetweenbias;

    ISAM2 *m_isam;

    void GpsCb(sensor_msgs::NavSatFixConstPtr fix);
    void ImuCb(sensor_msgs::ImuConstPtr imu);
    void FilterCb(imu_3dm_gx4::FilterOutputConstPtr fix);
    void GpsHelper();
    void diagnosticStatus(const ros::TimerEvent& time);

    void GetAccGyro(sensor_msgs::ImuConstPtr imu, Vector3 &acc, Vector3 &gyro);
  };
};

#endif /* IMU_GPS_H_ */
