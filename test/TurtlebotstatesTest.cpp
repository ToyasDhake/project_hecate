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
 * @file    TurtlebotstatesTest.cpp
 * @author  Shivam Akhauri, Toyas Dhake
 * @copyright 3-clause BSD
 * @brief Test cases for class Turtlebotstates
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "TurtlebotStates.hpp"

/**
 * @brief Test to verify if Obstacle detection is 
 * happening properly
 * Obtain laserscan sensor data and 
 * raise a flag if obstacle detected
 */
TEST(TESTTurtlebotState, checkObstacleDetection) {
    TurtlebotStates depthData;
    ros::NodeHandle nh;
    ros::Publisher pubScan = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
    ros::Subscriber depthBuffer = nh.subscribe<sensor_msgs::LaserScan>
                    ("/scan", 50, &TurtlebotStates::findLaserDepth, &depthData);

    sensor_msgs::LaserScan sensorData;
    sensorData.angle_min = -0.9;
    sensorData.angle_max = 0.9;
    sensorData.angle_increment = 0.001;
    sensorData.time_increment = 0.0;
    sensorData.range_min = 0.5;
    sensorData.range_max = 100.0;
    sensorData.ranges.resize(100);
    sensorData.intensities.resize(100);
    bool collision = false;
    int count = 0;
    depthData.returnLaserState();

    while (ros::ok()) {
        pubScan.publish(sensorData);
        if (depthData.flagCollision()) {
            collision = true;
            break;
        }
        ros::spinOnce();
        count++;
    }
    ASSERT_TRUE(collision);
}

/**
 * @brief check if flag is raised when obstacle 
 * distance is very less
 */
TEST(TESTTurtlebotState, checkDefaultflagCollisionValue) {
    TurtlebotStates turtlebotStates;
    ASSERT_FALSE(turtlebotStates.flagCollision());
}
