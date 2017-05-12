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
 * @file StateEstimator.cpp
 * @author Paul Drews <pdrews3@gatech.edu>
 * @date May 1, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief ROS node to fuse information sources and create an accurate state estimation
 *
 * @details Subscribes to the GPS, IMU, and wheel odometry topics, claculates
 * an estimate of the car's current state using GTSAM, and publishes that data.
 ***********************************************/



#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <vector>
#include "StateEstimator.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

using namespace gtsam;
// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::G; // GPS pose


// macro for getting the time stamp of a ros message
#define TIME(msg) ( (msg)->header.stamp.toSec() )

namespace autorally_core
{

  StateEstimator::StateEstimator() :
    Diagnostics("StateEstimator", "", ""),
    m_nh("~"),
    m_lastImuT(0.0),
    m_lastImuTgps(0.0),
    m_maxQSize(0),
    m_gpsOptQ(40),
    m_ImuOptQ(400),
    m_odomOptQ(100),
    m_gotFirstFix(false)
  {
    // temporary variables to retrieve parameters
    double accSigma, gyroSigma, initialVelNoise, initialBiasNoiseAcc, initialBiasNoiseGyro, initialRotationNoise,
        carXAngle, carYAngle, carZAngle, sensorX, sensorY, sensorZ, sensorXAngle, sensorYAngle, sensorZAngle,
        gravityMagnitude;

    m_nh.param<double>("InitialRotationNoise", initialRotationNoise, 1.0);
    m_nh.param<double>("InitialVelocityNoise", initialVelNoise, 0.1);
    m_nh.param<double>("InitialBiasNoiseAcc", initialBiasNoiseAcc, 1e-1);
    m_nh.param<double>("InitialBiasNoiseGyro", initialBiasNoiseGyro, 1e-2);
    m_nh.param<double>("AccelerometerSigma", accSigma, 6.0e-2);
    m_nh.param<double>("GyroSigma", gyroSigma, 2.0e-2);
    m_nh.param<double>("AccelBiasSigma", m_AccelBiasSigma, 2.0e-4);
    m_nh.param<double>("GyroBiasSigma", m_GyroBiasSigma, 3.0e-5);
    m_nh.param<double>("GPSSigma", m_gpsSigma, 0.07);
    m_nh.param<double>("SensorTransformX", sensorX, 0.0);
    m_nh.param<double>("SensorTransformY", sensorY, 0.0);
    m_nh.param<double>("SensorTransformZ", sensorZ, 0.0);
    m_nh.param<double>("SensorXAngle",  sensorXAngle, 0);
    m_nh.param<double>("SensorYAngle", sensorYAngle, 0);
    m_nh.param<double>("SensorZAngle",   sensorZAngle, 0);
    m_nh.param<double>("CarXAngle",  carXAngle, 0);
    m_nh.param<double>("CarYAngle",  carYAngle, 0);
    m_nh.param<double>("CarZAngle",  carZAngle, 0);
    m_nh.param<double>("Gravity",   gravityMagnitude, 9.8);
    m_nh.param<bool>("InvertX", m_invertx, false);
    m_nh.param<bool>("InvertY", m_inverty, false);
    m_nh.param<bool>("InvertZ", m_invertz, false);
    m_nh.param<double>("Imudt", m_imuDt, 1.0/200.0);

    double gpsx, gpsy, gpsz;
    m_nh.param<double>("GPSX",  gpsx, 0);
    m_nh.param<double>("GPSY",  gpsy, 0);
    m_nh.param<double>("GPSZ",  gpsz, 0);
    m_imuPgps = Pose3(Rot3(), Point3(gpsx, gpsy, gpsz));
    m_imuPgps.print("IMU->GPS");

    bool fixedInitialPose;
    double initialRoll, intialPitch, initialYaw;

    m_nh.param<bool>("FixedInitialPose", fixedInitialPose, false);
    m_nh.param<double>("initialRoll", initialRoll, 0);
    m_nh.param<double>("intialPitch", intialPitch, 0);
    m_nh.param<double>("initialYaw", initialYaw, 0);

    double latOrigin, lonOrigin, altOrigin;
    m_nh.param<bool>("FixedOrigin", m_fixedOrigin, false);
    m_nh.param<double>("latOrigin", latOrigin, 0);
    m_nh.param<double>("lonOrigin", lonOrigin, 0);
    m_nh.param<double>("altOrigin", altOrigin, 0);

    m_nh.param<bool>("UseOdom", m_usingOdom, false);
    m_nh.param<int>("FactorFrequency", m_frequency, 10);
    m_nh.param<double>("MaxGPSError", m_maxGPSError, 10);
    m_nh.param<double>("TimeWithoutGPS", m_timeWithoutGPS, 3);

    if (m_fixedOrigin)
      m_enu.Reset(latOrigin, lonOrigin, altOrigin);


    std::cout << "InitialRotationNoise " << initialRotationNoise << "\n"
    << "InitialVelocityNoise " << initialVelNoise << "\n"
    << "InitialBiasNoiseAcc " << initialBiasNoiseAcc << "\n"
    << "InitialBiasNoiseGyro " << initialBiasNoiseGyro << "\n"
    << "AccelerometerSigma " << accSigma << "\n"
    << "GyroSigma " << gyroSigma << "\n"
    << "AccelBiasSigma " << m_AccelBiasSigma << "\n"
    << "GyroBiasSigma " << m_GyroBiasSigma << "\n"
    << "GPSSigma " << m_gpsSigma << "\n"
    << "SensorTransformX " << sensorX << "\n"
    << "SensorTransformY " << sensorY << "\n"
    << "SensorTransformZ " << sensorZ << "\n"
    << "SensorXAngle " <<  sensorXAngle << "\n"
    << "SensorYAngle " << sensorYAngle << "\n"
    << "SensorZAngle " <<   sensorZAngle << "\n"
    << "CarXAngle " <<  carXAngle << "\n"
    << "CarYAngle " <<  carYAngle << "\n"
    << "CarZAngle " <<  carZAngle << "\n"
    << "Gravity " <<   gravityMagnitude << "\n";

    // Use an ENU frame
    m_preintegrationParams =  PreintegrationParams::MakeSharedU(gravityMagnitude);
    m_preintegrationParams->accelerometerCovariance = accSigma * I_3x3;
    m_preintegrationParams->gyroscopeCovariance = gyroSigma * I_3x3;
    m_preintegrationParams->integrationCovariance = 1e-5 * I_3x3;

    Vector biases((Vector(6) << 0, 0, 0, 0, 0, 0).finished());
    m_optimizedBias = imuBias::ConstantBias(biases);
    m_previousBias = imuBias::ConstantBias(biases);
    m_imuPredictor = boost::make_shared<PreintegratedImuMeasurements>(m_preintegrationParams,m_optimizedBias);

    m_optimizedTime = 0;

    imu_3dm_gx4::FilterOutputConstPtr ip;
    if (!fixedInitialPose) {
      while (!ip)
      {
        ROS_WARN("Waiting for valid initial pose");
        ip = ros::topic::waitForMessage<imu_3dm_gx4::FilterOutput>("filter", m_nh, ros::Duration(15));
      }
      m_initialPose = *ip;
    }
    else {
      ROS_WARN("Using fixed initial pose");
      Rot3 initialRotation = Rot3::Ypr(initialYaw, intialPitch, initialRoll);
      m_initialPose.orientation.w = initialRotation.quaternion()[0];
      m_initialPose.orientation.x = initialRotation.quaternion()[1];
      m_initialPose.orientation.y = initialRotation.quaternion()[2];
      m_initialPose.orientation.z = initialRotation.quaternion()[3];
      m_initialPose.bias.x = 0;
      m_initialPose.bias.y = 0;
      m_initialPose.bias.z = 0;
    }

    Rot3 initRot(Quaternion(m_initialPose.orientation.w, m_initialPose.orientation.x, m_initialPose.orientation.y, m_initialPose.orientation.z));

    m_bodyPSensor = Pose3(Rot3::RzRyRx(sensorXAngle, sensorYAngle, sensorZAngle),
        Point3(sensorX, sensorY, sensorZ));
    m_carENUPcarNED = Pose3(Rot3::RzRyRx(carXAngle, carYAngle, carZAngle), Point3());

    m_bodyPSensor.print("Body pose\n");
    m_carENUPcarNED.print("CarBodyPose\n");

    m_posePub = m_nh.advertise<nav_msgs::Odometry>("pose", 1);
    m_biasAccPub = m_nh.advertise<geometry_msgs::Point>("bias_acc", 1);
    m_biasGyroPub = m_nh.advertise<geometry_msgs::Point>("bias_gyro", 1);
    m_timePub = m_nh.advertise<geometry_msgs::Point>("time_delays", 1);
    m_statusPub = m_nh.advertise<autorally_msgs::stateEstimatorStatus>("status", 1);

    ISAM2Params params;
    params.factorization = ISAM2Params::QR;
    m_isam = new ISAM2(params);

    // prior on the first pose
    priorNoisePose = noiseModel::Diagonal::Sigmas(
         (Vector(6) << initialRotationNoise, initialRotationNoise, 3*initialRotationNoise,
             m_gpsSigma, m_gpsSigma, m_gpsSigma).finished());

     // Add velocity prior
     priorNoiseVel = noiseModel::Diagonal::Sigmas(
         (Vector(3) << initialVelNoise, initialVelNoise, initialVelNoise).finished());

     // Add bias prior
     priorNoiseBias = noiseModel::Diagonal::Sigmas(
         (Vector(6) << initialBiasNoiseAcc,
             initialBiasNoiseAcc,
             initialBiasNoiseAcc,
             initialBiasNoiseGyro,
             initialBiasNoiseGyro,
             initialBiasNoiseGyro).finished());

     sigma_acc_bias_c << m_AccelBiasSigma,  m_AccelBiasSigma,  m_AccelBiasSigma;
     sigma_gyro_bias_c << m_GyroBiasSigma, m_GyroBiasSigma, m_GyroBiasSigma;
     noiseModelBetweenbias_sigma = (Vector(6) << sigma_acc_bias_c, sigma_gyro_bias_c).finished();
     noiseModelBetweenbias = noiseModel::Diagonal::Sigmas((noiseModelBetweenbias_sigma));

     m_gpsSub = m_nh.subscribe("gps", 300, &StateEstimator::GpsCallback, this);
     m_imuSub = m_nh.subscribe("imu", 600, &StateEstimator::ImuCallback, this);
     if (m_usingOdom)
       m_odomSub = m_nh.subscribe("wheel_odom", 300, &StateEstimator::WheelOdomCallback, this);

     boost::thread optimizer(&StateEstimator::GpsHelper,this);
  }

  StateEstimator::~StateEstimator()
  {}

  void StateEstimator::GpsCallback(sensor_msgs::NavSatFixConstPtr fix)
  {
    if (!m_gpsOptQ.pushNonBlocking(fix))
      ROS_WARN("Dropping a GPS measurement due to full queue!!");
  }

  void StateEstimator::GetAccGyro(sensor_msgs::ImuConstPtr imu, Vector3 &acc, Vector3 &gyro)
  {
    double accx, accy, accz;
    if (m_invertx) accx = -imu->linear_acceleration.x;
    else accx = imu->linear_acceleration.x;
    if (m_inverty) accy = -imu->linear_acceleration.y;
    else accy = imu->linear_acceleration.y;
    if (m_invertz) accz = -imu->linear_acceleration.z;
    else accz = imu->linear_acceleration.z;
    acc = Vector3(accx, accy, accz);

    double gx, gy, gz;
    if (m_invertx) gx = -imu->angular_velocity.x;
    else gx = imu->angular_velocity.x;
    if (m_inverty) gy = -imu->angular_velocity.y;
    else gy = imu->angular_velocity.y;
    if (m_invertz) gz = -imu->angular_velocity.z;
    else gz = imu->angular_velocity.z;

    gyro = Vector3(gx, gy, gz);
  }


  void StateEstimator::GpsHelper()
  {
    ros::Rate loop_rate(10);
    bool gotFirstFix = false; // doesn't need to be a member variable
    double startTime;
    int odomKey = 1;
    int imuKey = 1;
    imuBias::ConstantBias prevBias;
    Vector3 prevVel = (Vector(3) << 0.0,0.0,0.0).finished();
    Pose3 prevPose;
    unsigned char status = autorally_msgs::stateEstimatorStatus::OK;


    while (ros::ok())
    {
      bool optimize = false;

      if (!gotFirstFix)
      {
        sensor_msgs::NavSatFixConstPtr fix = m_gpsOptQ.popBlocking();
        startTime = TIME(fix);
        if (m_usingOdom)
          m_lastOdom = m_odomOptQ.popBlocking();

        NonlinearFactorGraph newFactors;
        Values newVariables;
        gotFirstFix = true;

        double E, N, U;
        if (!m_fixedOrigin)
        {
          m_enu.Reset(fix->latitude, fix->longitude, fix->altitude);
          E = 0; N = 0; U = 0; // we're choosing this as the origin
        }
        else
        {
          // we are given an origin
          m_enu.Forward(fix->latitude, fix->longitude, fix->altitude, E, N, U);
        }

        // Add prior factors on pose, vel and bias
        Rot3 initialOrientation = Rot3::Quaternion(m_initialPose.orientation.w,
            m_initialPose.orientation.x,
            m_initialPose.orientation.y,
            m_initialPose.orientation.z);
        std::cout << "Initial orientation" << std::endl;
        std::cout << m_bodyPSensor.rotation() * initialOrientation * m_carENUPcarNED.rotation() << std::endl;
        Pose3 x0(m_bodyPSensor.rotation() * initialOrientation * m_carENUPcarNED.rotation(),
            Point3(E, N, U));
        prevPose = x0;
        PriorFactor<Pose3> priorPose(X(0), x0, priorNoisePose);
        newFactors.add(priorPose);
        PriorFactor<Vector3> priorVel(V(0), Vector3(0, 0, 0), priorNoiseVel);
        newFactors.add(priorVel);
        Vector biases((Vector(6) << 0, 0, 0, m_initialPose.bias.x,
            -m_initialPose.bias.y, -m_initialPose.bias.z).finished());
        prevBias = imuBias::ConstantBias(biases);
        PriorFactor<imuBias::ConstantBias> priorBias(B(0), imuBias::ConstantBias(biases), priorNoiseBias);
        newFactors.add(priorBias);

        //Factor for imu->gps translation
        BetweenFactor<Pose3> imuPgpsFactor(X(0), G(0), m_imuPgps,
            noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
        newFactors.add(imuPgpsFactor);

        // add prior values on pose, vel and bias
        newVariables.insert(X(0), x0);
        newVariables.insert(V(0), Vector3(0, 0, 0));
        newVariables.insert(B(0), imuBias::ConstantBias(biases));
        newVariables.insert(G(0), x0.compose(m_imuPgps));

        m_isam->update(newFactors, newVariables);
        //Read IMU measurements up to the first GPS measurement
        m_lastIMU = m_ImuOptQ.popBlocking();
        //If we only pop one, we need some dt
        m_lastImuTgps = m_lastIMU->header.stamp.toSec() - 0.005;
        while(m_lastIMU->header.stamp.toSec() < TIME(fix))
        {
          m_lastImuTgps = m_lastIMU->header.stamp.toSec();
          m_lastIMU = m_ImuOptQ.popBlocking();
        }

        loop_rate.sleep();
      }
      else
      {
        NonlinearFactorGraph newFactors;
        Values newVariables;


        // if available, add any odom factors
        while (m_odomOptQ.size() > 0 && (TIME(m_odomOptQ.back()) > (startTime + odomKey * 0.1)))
        {
          double prevTime = startTime + (odomKey-1) * 0.1;
          newFactors.add(integrateWheelOdom(prevTime, prevTime+0.1, odomKey++));
        }

        // add GPS measurements
        while (m_gpsOptQ.size() > 0) // && (TIME(m_gpsOptQ.front()) < ((imuKey-1)*0.1+startTime+1e-2)))
        {
          sensor_msgs::NavSatFixConstPtr fix = m_gpsOptQ.popBlocking();
          double timeDiff = (TIME(fix) - startTime) / 0.1;
          int key = round(timeDiff);
          if (std::abs(timeDiff - key) < 1e-4)
          {
            // this is a gps message for a factor
            // TODO how to check if this message is bad - we have gotten rid of the imu messaes to predict
            double E,N,U;
            m_enu.Forward(fix->latitude, fix->longitude, fix->altitude, E, N, U);
            SharedDiagonal gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(m_gpsSigma, m_gpsSigma, 3.0 * m_gpsSigma));
            GPSFactor gpsFactor(G(key), Point3(E, N, U), gpsNoise);
            newFactors.add(gpsFactor);
            BetweenFactor<Pose3> imuPgpsFactor(X(key), G(key), m_imuPgps,
                noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
            newFactors.add(imuPgpsFactor);
          }
        }

        // add IMU measurements
        while (m_ImuOptQ.size() > 0 && (TIME(m_ImuOptQ.back()) > (startTime + imuKey * 0.1)))
        {
          double curTime = startTime + imuKey * 0.1;
          PreintegratedImuMeasurements pre_int_data(m_preintegrationParams, m_previousBias);
          while(TIME(m_lastIMU) < curTime)
          {
            Vector3 acc, gyro;
            GetAccGyro(m_lastIMU, acc, gyro);
            double imuDT = TIME(m_lastIMU) - m_lastImuTgps;
            m_lastImuTgps = TIME(m_lastIMU);
            pre_int_data.integrateMeasurement(acc, gyro, imuDT);
            m_lastIMU = m_ImuOptQ.popBlocking();
          }
          // adding the integrated IMU measurements to the factor graph
          ImuFactor imuFactor(X(imuKey-1), V(imuKey-1), X(imuKey), V(imuKey), B(imuKey-1), pre_int_data);
          newFactors.add(imuFactor);
          newFactors.add(BetweenFactor<imuBias::ConstantBias>(B(imuKey-1), B(imuKey), imuBias::ConstantBias(),
              noiseModel::Diagonal::Sigmas( sqrt(pre_int_data.deltaTij()) * noiseModelBetweenbias_sigma)));

          // Predict forward to get an initial estimate for the pose and velocity
          NavState curNavState(prevPose, prevVel);
          NavState nextNavState = pre_int_data.predict(curNavState, prevBias);
          newVariables.insert(X(imuKey), nextNavState.pose());
          newVariables.insert(V(imuKey), nextNavState.v());
          newVariables.insert(B(imuKey), m_previousBias);
          newVariables.insert(G(imuKey), nextNavState.pose().compose(m_imuPgps));
          // TODO figure out if I should update first - this might propagate error, but will almost certainly not happen often
          prevPose = nextNavState.pose();
          prevVel = nextNavState.v();
          ++imuKey;
          optimize = true;
        }


        // if we processed imu - then we can optimize the state
        if (optimize)
        {
          try
          {
            m_isam->update(newFactors, newVariables);
            Pose3 nextState = m_isam->calculateEstimate<Pose3>(X(imuKey-1));

            prevPose = nextState;
            prevVel = m_isam->calculateEstimate<Vector3>(V(imuKey-1));
            prevBias = m_isam->calculateEstimate<imuBias::ConstantBias>(B(imuKey-1));
            //std::cout << m_isam->marginalCovariance(X(imuKey)) << std::endl << std::endl;

            double curTime = startTime + (imuKey-1) * 0.1;
            diag_ok("Still ok!");

            {
              boost::mutex::scoped_lock guard(m_optimizedStateMutex);
              m_optimizedState = NavState(prevPose, prevVel);
              m_optimizedBias = prevBias;
              m_optimizedTime = curTime;
              m_status = status;
            }

            nav_msgs::Odometry poseNew;
            poseNew.header.stamp = ros::Time(curTime);

            geometry_msgs::Point ptAcc;
            ptAcc.x = prevBias.vector()[0];
            ptAcc.y = prevBias.vector()[1];
            ptAcc.z = prevBias.vector()[2];

            geometry_msgs::Point ptGyro;
            ptGyro.x = prevBias.vector()[3];
            ptGyro.y = prevBias.vector()[4];
            ptGyro.z = prevBias.vector()[5];

            m_biasAccPub.publish(ptAcc);
            m_biasGyroPub.publish(ptGyro);
          }
          catch(gtsam::IndeterminantLinearSystemException ex)
          {
            ROS_ERROR("Encountered Indeterminant System Error!");
            diag_error("State estimator has encountered indeterminant system error");
            status = autorally_msgs::stateEstimatorStatus::ERROR;
            {
              boost::mutex::scoped_lock guard(m_optimizedStateMutex);
              m_status = status;
            }
          }
        }
        loop_rate.sleep();
      }
    }
  }


  void StateEstimator::ImuCallback(sensor_msgs::ImuConstPtr imu)
  {
    double dt;
    if (m_lastImuT == 0) dt = 0.005;
    else dt = TIME(imu) - m_lastImuT;

    m_lastImuT = TIME(imu);
    ros::Time before = ros::Time::now();

    // Push the IMU measurement to the optimization thread
    int qSize = m_ImuOptQ.size();
    if (qSize > m_maxQSize)
      m_maxQSize = qSize;
    if (!m_ImuOptQ.pushNonBlocking(imu))
      ROS_WARN("Dropping an IMU measurement due to full queue!!");

    // Each time we get an imu measurement, calculate the incremental pose from the last GTSAM pose
    m_imuMeasurements.push_back(imu);
    //Grab the most current optimized state
    double optimizedTime;
    NavState optimizedState;
    imuBias::ConstantBias optimizedBias;
    unsigned char status;
    {
      boost::mutex::scoped_lock guard(m_optimizedStateMutex);
      optimizedState = m_optimizedState;
      optimizedBias = m_optimizedBias;
      optimizedTime = m_optimizedTime;
      status = m_status;
    }
    if (optimizedTime == 0) return; // haven't optimized first state yet

    bool newMeasurements = false;
    int numImuDiscarded = 0;
    double imuQPrevTime;
    Vector3 acc, gyro;
    while (!m_imuMeasurements.empty() && (TIME(m_imuMeasurements.front()) < optimizedTime))
    {
      imuQPrevTime = TIME(m_imuMeasurements.front());
      m_imuMeasurements.pop_front();
      newMeasurements = true;
      numImuDiscarded++;
    }

    if(newMeasurements)
    {
      //We need to reset integration and iterate through all our IMU measurements
      m_imuPredictor->resetIntegration();
      int numMeasurements = 0;
      for (auto it=m_imuMeasurements.begin(); it!=m_imuMeasurements.end(); ++it)
      {
        double dt_temp =  TIME(*it) - imuQPrevTime;
        imuQPrevTime = TIME(*it);
        GetAccGyro(*it, acc, gyro);
        m_imuPredictor->integrateMeasurement(acc, gyro, dt_temp);
        numMeasurements++;
        // ROS_INFO("IMU time %f, dt %f", (*it)->header.stamp.toSec(), dt_temp);
      }
      // ROS_INFO("Resetting Integration, %d measurements integrated, %d discarded", numMeasurements, numImuDiscarded);
    }
    else
    {
      //Just need to add the newest measurement, no new optimized pose
      GetAccGyro(imu, acc, gyro);
      m_imuPredictor->integrateMeasurement(acc, gyro, dt);//m_bodyPSensor);
      // ROS_INFO("Integrating %f, dt %f", m_lastImuT, dt);
    }

    // predict next state given the imu measurements
    NavState currentPose = m_imuPredictor->predict(optimizedState, optimizedBias);
    nav_msgs::Odometry poseNew;
    poseNew.header.stamp = imu->header.stamp;

    Vector4 q = currentPose.quaternion().coeffs();
    poseNew.pose.pose.orientation.x = q[0];
    poseNew.pose.pose.orientation.y = q[1];
    poseNew.pose.pose.orientation.z = q[2];
    poseNew.pose.pose.orientation.w = q[3];

    poseNew.pose.pose.position.x = currentPose.position().x();
    poseNew.pose.pose.position.y = currentPose.position().y();
    poseNew.pose.pose.position.z = currentPose.position().z();

    poseNew.twist.twist.linear.x = currentPose.velocity().x();
    poseNew.twist.twist.linear.y = currentPose.velocity().y();
    poseNew.twist.twist.linear.z = currentPose.velocity().z();
    
    poseNew.twist.twist.angular.x = gyro.x() + optimizedBias.gyroscope().x();
    poseNew.twist.twist.angular.y = gyro.y() + optimizedBias.gyroscope().y();
    poseNew.twist.twist.angular.z = gyro.z() + optimizedBias.gyroscope().z();

    poseNew.child_frame_id = "base_link";
    poseNew.header.frame_id = "odom";

    m_posePub.publish(poseNew);

    ros::Time after = ros::Time::now();
    geometry_msgs::Point delays;
    delays.x = TIME(imu);
    delays.y = (ros::Time::now() - imu->header.stamp).toSec();
    delays.z = TIME(imu) - optimizedTime;
    m_timePub.publish(delays);

    // publish the status of the estimate - set in the gpsHelper thread
    autorally_msgs::stateEstimatorStatus statusMsgs;
    statusMsgs.header.stamp = imu->header.stamp;
    statusMsgs.status = status;
    m_statusPub.publish(statusMsgs);
    return;
  }

  void StateEstimator::WheelOdomCallback(nav_msgs::OdometryConstPtr odom)
  {
      if (!m_odomOptQ.pushNonBlocking(odom))
        ROS_WARN("Dropping an wheel odometry measurement due to full queue!!");
  }


  BetweenFactor<Pose3> StateEstimator::integrateWheelOdom(double prevTime, double stopTime, int curKey)
  {
    double x=0, y=0, theta=0, xVariance=0, thetaVariance=0, dt=0, lastTimeUsed=prevTime;

    while (lastTimeUsed != stopTime)
    {
      if (m_odomOptQ.size() != 0 && m_odomOptQ.front()->header.stamp.toSec() < stopTime)
      {
        m_lastOdom = m_odomOptQ.popBlocking();
        dt = m_lastOdom->header.stamp.toSec() - lastTimeUsed;
        lastTimeUsed = m_lastOdom->header.stamp.toSec();
      }
      else
      {
        dt = stopTime - lastTimeUsed;
        lastTimeUsed = stopTime;
      }

      // the local frame velocities
      double vx = m_lastOdom->twist.twist.linear.x;
      double vy = m_lastOdom->twist.twist.linear.y;
      // update the relative position from the initial
      x += vx*dt*cos(theta) - vy*dt*sin(theta);
      y += vx*dt*sin(theta) + vy*dt*cos(theta);
      theta += dt*m_lastOdom->twist.twist.angular.z;
      xVariance += dt *  m_lastOdom->twist.covariance[0];
      thetaVariance += dt*m_lastOdom->twist.covariance[35];
    }

    Pose3 betweenPose = Pose3(Rot3::Rz(theta), Point3(x, y, 0.0));
    return BetweenFactor<Pose3>(X(curKey-1), X(curKey), betweenPose, noiseModel::Diagonal::Sigmas(
          (Vector(6) << xVariance,100,100,100,100,thetaVariance).finished()));
  }

  void StateEstimator::diagnosticStatus(const ros::TimerEvent& /*time*/)
  {
    //Don't do anything
    //diag_info("Test");
  }

};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "StateEstimator");
  //ros::NodeHandle n;
  autorally_core::StateEstimator wpt;
  ros::spin();
}
