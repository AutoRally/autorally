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
 * @file diagnosticsTest.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date August 21, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief Unit tests for Diagnostics
 *
 ***********************************************/
#include <gtest/gtest.h>

/* This is an ugly line included simply so that I can verify that function calls
 * with no return values can be tested without subscribing to the ROS messages
 * they trigger.
 */
#define private public
#include <infrastructure/Diagnostics.h>

#include <ros/ros.h>
#include <ros/time.h>

/**
 *  @class DiagnosticsTest
 *  @brief Wrapper class for SerialSensorInterface for unit testing purposes
 *  
 *  @see Diagnostics
 *
 */
class DiagnosticsTest : public Diagnostics
{
 public:
  DiagnosticsTest(ros::NodeHandle &nh) :
    Diagnostics(nh, "nodeName", "hardwareID", "hardwareLocation")
  {}
  
  void diagnosticStatus(const ros::TimerEvent& time)
  {}
  // virtual void TearDown() {}
};

/**
  * @test Diagnostics initialization unit test
  *
  */
TEST(DagnosticsBasic, initialization)
{
  ros::NodeHandle nh;
  DiagnosticsTest a(nh);

  EXPECT_EQ(a.m_name, "nodeName");
  EXPECT_EQ(a.m_hardwareLocation, "hardwareLocation");
}

/**
  * @test Diagnostics unit test covering the control of the overall level of a
  *       diagnostics message
  */
TEST(DagnosticsTest, overallLevel)
{
  ros::NodeHandle nh;
  DiagnosticsTest a(nh);
  
  //set overall level
  EXPECT_EQ(a.m_overallLevel, diagnostic_msgs::DiagnosticStatus::OK);
  a.WARN();
  EXPECT_EQ(a.m_overallLevel, diagnostic_msgs::DiagnosticStatus::WARN);
  a.ERROR();
  EXPECT_EQ(a.m_overallLevel, diagnostic_msgs::DiagnosticStatus::ERROR);
  
  //force diagnostic publication
  ros::TimerEvent s;
  a.diagUpdate(s);
  
  //check overall status is unchanged, then change it
  EXPECT_EQ(a.m_overallLevel, diagnostic_msgs::DiagnosticStatus::ERROR);
  a.OK();
  EXPECT_EQ(a.m_overallLevel, diagnostic_msgs::DiagnosticStatus::OK);
}

/**
  * @test Diagnostics unit test covering messages added with status OK
  *
  */
TEST(DagnosticsTest, okMessages)
{

  ros::NodeHandle nh;
  DiagnosticsTest a(nh);
  
  EXPECT_TRUE(a.m_diagMsgs.empty());
  a.diag_ok("check");
  EXPECT_EQ(a.m_diagMsgs.size(), 1);
  EXPECT_EQ(a.m_diagMsgs["check"], diagnostic_msgs::DiagnosticStatus::OK);
  
  //force diagnostic message to be sent
  ros::TimerEvent s;
  a.diagUpdate(s);
  
  //make sure queue is now empty
  EXPECT_TRUE(a.m_diagMsgs.empty());
}

/**
  * @test Diagnostics unit test covering messages added with status WARN
  *
  */
TEST(DagnosticsTest, warnMessages)
{
  ros::NodeHandle nh;
  DiagnosticsTest a(nh);
  
  EXPECT_TRUE(a.m_diagMsgs.empty());
  a.diag_warn("checkWARN");
  EXPECT_EQ(a.m_diagMsgs.size(), 1);
  EXPECT_EQ(a.m_diagMsgs["checkWARN"], diagnostic_msgs::DiagnosticStatus::WARN);
  
  //force diagnostic message to be sent
  ros::TimerEvent s;
  a.diagUpdate(s);
  
  //make sure queue is now empty
  EXPECT_TRUE(a.m_diagMsgs.empty());
}

/**
  * @test Diagnostics unit test covering messages added with status ERROR
  *
  */
TEST(DagnosticsTest, errorMessages)
{
  ros::NodeHandle nh;
  DiagnosticsTest a(nh);
  
  EXPECT_TRUE(a.m_diagMsgs.empty());
  a.diag_error("12232");
  EXPECT_EQ(a.m_diagMsgs.size(), 1);
  EXPECT_EQ(a.m_diagMsgs["12232"], diagnostic_msgs::DiagnosticStatus::ERROR);
  
  //force diagnostic message to be sent
  ros::TimerEvent s;
  a.diagUpdate(s);
  
  //make sure queue is now empty
  EXPECT_TRUE(a.m_diagMsgs.empty()); 
}

/**
  * @test Diagnostics unit test covering messages involving key value pairs of
  *       type std::string
  */
TEST(DagnosticsTest, stringMessages)
{
  ros::NodeHandle nh;
  DiagnosticsTest a(nh);
  
  EXPECT_TRUE(a.m_diags.empty());
  a.diag("key", "value");
  EXPECT_EQ(a.m_diags.size(), 1);
  EXPECT_EQ(a.m_diags["key"], "value");
  
  a.diag("!#(<:""", "@$($@&*^$@");
  EXPECT_EQ(a.m_diags.size(), 2);
  EXPECT_EQ(a.m_diags["!#(<:"""], "@$($@&*^$@");
  
  //force diagnostic message to be sent
  ros::TimerEvent s;
  a.diagUpdate(s);
  
  //make sure queue is now empty
  EXPECT_TRUE(a.m_diags.empty()); 
}

/**
  * @test Diagnostics unit test covering the correct behavior when multiple
  *       messages are added of each type ( [msg,level] and [key,value])
  */
TEST(DagnosticsTest, multipleLevels)
{
  ros::NodeHandle nh;
  DiagnosticsTest a(nh);
  
  EXPECT_TRUE(a.m_diagMsgs.empty());
  a.diag_warn("warn");
  a.diag_ok("ok");
  a.diag_error("error");
  a.diag("blah","junk");
  a.diag("xawfe","junk");
  
  EXPECT_EQ(a.m_diagMsgs.size(), 3);
  EXPECT_EQ(a.m_diags.size(), 2);
  //force diagnostic message to be sent
  ros::TimerEvent s;
  a.diagUpdate(s);
  
  //make sure queue is now empty
  EXPECT_TRUE(a.m_diagMsgs.empty());
  EXPECT_TRUE(a.m_diags.empty());
}

/**
  * @test Diagnostics unit test covering the behavior when multiple messagges
  *       with the same msg or key are added, only the most recent is maintained
  */
TEST(DagnosticsTest, mostRecentLevel)
{
  ros::NodeHandle nh;
  DiagnosticsTest a(nh);
  
  EXPECT_TRUE(a.m_diagMsgs.empty());
  a.diag_error("test");
  EXPECT_EQ(a.m_diagMsgs.size(), 1);
  EXPECT_EQ(a.m_diagMsgs["test"], diagnostic_msgs::DiagnosticStatus::ERROR);
  
  a.diag_ok("test");
  EXPECT_EQ(a.m_diagMsgs.size(), 1);
  EXPECT_EQ(a.m_diagMsgs["test"], diagnostic_msgs::DiagnosticStatus::OK);
  
  a.diag_warn("test");
  EXPECT_EQ(a.m_diagMsgs.size(), 1);
  EXPECT_EQ(a.m_diagMsgs["test"], diagnostic_msgs::DiagnosticStatus::WARN);
  
  a.diag("key","old");
  a.diag("key","new");
  EXPECT_EQ(a.m_diags.size(), 1);
  EXPECT_EQ(a.m_diags["key"], "new");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  
  ros::init(argc, argv, "diagnostics_validation");
  
  return RUN_ALL_TESTS();
}
