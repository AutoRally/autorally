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
 * @file pololuMicroMaestroTest.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date August 21, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief Unit tests for PololuMicroMaestro
 *
 ***********************************************/
#include <gtest/gtest.h>

/* This is an ugly line included simply so that I can verify that function calls
 * with no return values can be tested without subscribing to the ROS messages
 * they trigger.
 */
#define private public
#include "infrastructure/PololuMicroMaestro.h"

/**
  * @test PololuMicroMaestro initialization unit test (empty)
  *
  */
TEST(PololuMicroMaestroBasic, initialization)
{
  ros::NodeHandle nh;
  
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  
  ros::init(argc, argv, "pololuMicroMaestro_validation");
  
  return RUN_ALL_TESTS();
}
