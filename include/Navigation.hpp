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
* @file Navigation.hpp
* @author Shivam Akhauri, Toyas Dhake
* @date 27 November 2019
* @copyright BSD 3-clause, 2019 Shivam Akhauri,Toyas Dhake
* @brief Header for the robot autonomous of the robot
*/
#ifndef _HOME_TOYAS_CATKIN_WS_SRC_PROJECT_HECATE_INCLUDE_NAVIGATION_HPP_
#define _HOME_TOYAS_CATKIN_WS_SRC_PROJECT_HECATE_INCLUDE_NAVIGATION_HPP_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "TurtlebotStates.hpp"
#include "QLearning.hpp"
#include "nav_msgs/Odometry.h"

/**
 * @brief Class Navigation
 * This class contains members to generate linear and angular
 * velocities to the turtulebot based on the depth from 
 * the obstacle information received from the depthCalculator
*/
class Navigation {
 private:
    QLearning qLearning;
    ros::NodeHandle nh;
    geometry_msgs::Twist twistMessage;
    ros::Publisher velocityPublisher;
    ros::Subscriber distanceList;
    ros::Subscriber sub_odom;
    TurtlebotStates turtlebotStates;
    int reward;

 public:
    double x, y, z, roll, pitch, yaw, x_goal, y_goal;
    /**
    * @brief constructor Navigation class
    * @param none
    * @return none
    * initializes the publisher and subsciber
    * initialize the value of odometry
    * initialize the liner and angular speed
    */
    Navigation();

   /**
    * @brief destructor Navigation class
    * @param none
    * @return none
    * Destructor for the navigation clas
    */
    ~Navigation();

    /**
    * @brief function testRobot
    * @param path std::string
    * @return none
    * Runs the inferece code 
    * the bot uses the trained model to navigate
    */
    void testRobot(std::string path, double ix, double iy,
                                                          double fx, double fy);
    /**
    * @brief function trainRobot
    * @param path std::string
    * @return none
    * training of the agent by receiving states
    * perform actions in that states and receive rewards
    */
    void trainRobot(std::string path);
    /**
    * @brief function demoAction
    * @param std::vector<int> state
    * @return int stateIndex
    * mapping the vector to the state in rl table
    */
    int getStateIndex(std::vector<int> state);
    /**
    * @brief function demoAction
    * @param int action
    * @return none
    * publishes linear and angular velocities to the turtlebot
    */
    void action(int action, bool &colStatus, int &reward, int &nextState);
     /**
    * @brief function environmentPause
    * @param none
    * @return none
    * pauses the gazebo environment
    */
    void environmentReset();
    /**
    * @brief function demoAction
    * @param int action
    * @return none
    * publishes linear and angular velocities to the turtlebot
    */
    void demoAction(int action);
    void dom(const nav_msgs::Odometry::ConstPtr &msg);
};

#endif  // _HOME_TOYAS_CATKIN_WS_SRC_PROJECT_HECATE_INCLUDE_NAVIGATION_HPP_
