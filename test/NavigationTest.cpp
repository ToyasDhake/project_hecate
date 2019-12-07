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
#include "QLearning.hpp"




TEST(TESTNavigation, checkForCorrectStateIndex) {
  Navigation Nav;
  std::vector<int> state = {2, 2, 2, 2};
  ASSERT_EQ(259, Nav.getStateIndex(state));
}

TEST(TESTNavigation, checkForTestRobot) {
  Navigation navigation;
  navigation.x_goal = 1;
  navigation.y_goal = 1;
  std::vector<int> state;
  QLearning qLearning;
  qLearning.getQtable("Path");
  ros::Rate loop_rate(10);
  navigation.testRobot(1,2,2, qLearning, state, loop_rate);
  navigation.x = 1;
  navigation.y = 1;
  navigation.testRobot(1,2,2, qLearning, state, loop_rate);
  navigation.x = 2;
  navigation.y = 2;
  navigation.testRobot(1,2,2, qLearning, state, loop_rate);
  EXPECT_NEAR(2, navigation.x, 0.7);
}

TEST(TESTNavigation, checkForTrainRobot) {
  Navigation navigation;
  int highestReward = 0;
  int episodeCount = 0;
  int totalEpisode = 1;  
  int nextStateIndex;
  ros::Rate loop_rate(10);
  navigation.trainRobot("path", highestReward, episodeCount, totalEpisode, nextStateIndex, loop_rate, 1);
  navigation.trainRobot("path", highestReward, episodeCount, totalEpisode, nextStateIndex, loop_rate, 1);
  navigation.trainRobot("path", highestReward, episodeCount, totalEpisode, nextStateIndex, loop_rate, 1);
  EXPECT_FALSE(false);
}
