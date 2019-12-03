/**
BSD 3-Clause License
Copyright (c) 2019, Shivam Akhauri,Toyas Dhake
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

/**
 * @file    NavigationTest.cpp
 * @author  Shivam Akhauri, Toyas Dhake
 * @copyright 3-clause BSD
 * @brief Test cases for class Navigation
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include "Navigation.hpp"



class TestVelocityOdom {
 private:
     geometry_msgs::Pose pose;
     geometry_msgs::Twist odom;
 public:
     void testVelocityPublish(const geometry_msgs::Twist::ConstPtr& msg) {
             odom = *msg;
     }
};

TEST(TESTNavigation, verifyOrientationofRobotTowardsGoal) {
  Navigation Nav;
  double ix = 2;
  double iy = 2;
  double fx = 5;
  double fy = 4;
  ASSERT_NE(0, Nav.testRobot("model.csv", ix, iy, fx, fy));
}


TEST(TESTNavigation, checkForCorrectStateIndex) {
  Navigation Nav;
  std::vector<int> state = {2, 2, 2, 2};
  ASSERT_EQ(259, Nav.getStateIndex(state));
}


TEST(TESTNavigation, checkIfVelocityPublished) {
    ros::NodeHandle nh;
    TestVelocityOdom temp;
    Navigation Nav;
    ros::Rate loop_rate(10);
    ros::Subscriber sub = nh.subscribe("/mobile_base/commands/velocity", 1000,
                                      &TestVelocityOdom::testVelocityPublish,
                                      &temp);
    loop_rate.sleep();
    EXPECT_EQ(1, sub.getNumPublishers());
}

TEST(TESTNavigation, checkRobotMovementForChoosenAction) {
    Navigation Nav;
    int action = 0; 
    EXPECT_EQ(0.5, Nav.demoAction(action));
}

// TEST(TESTNavigation, checkNewTrainRewardObtainedForAction) {
//    Navigation Nav;
//    ASSERT_GT(0, Nav.trainRobot("model.csv"));
//}
TEST(TESTNavigation, testIntializationError) {
    EXPECT_NO_FATAL_FAILURE(Navigation nav);
}



