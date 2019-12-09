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
* @file Navigation.cpp
* @author Shivam Akhauri, Toyas Dhake
* @date 27 November 2019
* @copyright BSD 3-clause, 2019 Shivam Akhauri,Toyas Dhake
* @brief Code for autonoumous naigation of the robot 
* from a user defined start point to a stop point
*/
#include <tf/transform_listener.h>
#include <cmath>
#include <boost/range/irange.hpp>
#include "Navigation.hpp"

void Navigation::dom(const nav_msgs::Odometry::ConstPtr &msg) {
    // Receive data from odometry
    // Obtain the x coordinates
    x = msg->pose.pose.position.x;
    // Obtain the y coordinates
    y = msg->pose.pose.position.y;
    // Obtain the z coordinates
    z = msg->pose.pose.position.z;
    tf::Quaternion q(
        // To obtain yaw
        msg->pose.pose.orientation.x,
        // To obtain pitch
        msg->pose.pose.orientation.y,
        // To obtain roll
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

Navigation::Navigation() {
    // Set initial yaw
    yaw = 0.0;
    // Advertise the twist
    velocityPublisher = nh.advertise<geometry_msgs::Twist>
                                          ("/mobile_base/commands/velocity", 1);
    distanceList = nh.subscribe<sensor_msgs::LaserScan>("/scan", 50,
                            &TurtlebotStates::findLaserDepth, &turtlebotStates);

    // Initialize the velocities
    twistMessage.linear.x = 0.0;
    twistMessage.linear.y = 0.0;
    twistMessage.linear.z = 0.0;
    twistMessage.angular.x = 0.0;
    twistMessage.angular.y = 0.0;
    twistMessage.angular.z = 0.0;
    // Subscribe to odometry of the turtlebot
    sub_odom = nh.subscribe("odom", 10, &Navigation::dom, this);
    // Publish velocity values for turtlebot
    velocityPublisher.publish(twistMessage);
}

Navigation::~Navigation() {
    // Stop the Linear robot motion
    twistMessage.linear.x = 0.0;
    twistMessage.linear.y = 0.0;
    twistMessage.linear.z = 0.0;
    // Stop the robot rotation
    twistMessage.angular.x = 0.0;
    twistMessage.angular.y = 0.0;
    twistMessage.angular.z = 0.0;
    // Publish velocity
    velocityPublisher.publish(twistMessage);
}

void Navigation::trainRobot(std::string path, int &highestReward,
                                int &episodeCount, int totalEpisode,
                                int &nextStateIndex, ros::Rate loop_rate,
                                int innerLoopLimit) {
    // Set high epsilon values to train a greedy approach
    double epsilonDiscount = 0.99;
    // Initialize turtlebot states
    std::vector<int> state;
    // Check number of epochs done in the episode
    if (episodeCount < totalEpisode) {
        // Initialize collision flag
        bool collision = false;
        // Initialize rewards
        int cumulatedReward = 0;
        // Reset the environment before episode starts
        environmentReset();
        // Epsilon should not go below 0.05
        if (qLearning.getEpsilon() > 0.05) {
            qLearning.setEpsilon(qLearning.getEpsilon() * epsilonDiscount);
        }
        int stateIndex = 0;
        // Obtain the sensor data
        state = turtlebotStates.returnLaserState();
        // Map sensor data with state index
        stateIndex = getStateIndex(state);
        // Initilize epoch count
        int innerLoopCount = 0;
        // Number of iterations per epoch
        while (innerLoopCount < innerLoopLimit) {
            int chosenAction;
            // Predict the action based on boltzmann equation
            chosenAction = qLearning.chooseAction(stateIndex);
            // Make the turtlebot perform the action
            action(chosenAction, collision, reward, nextStateIndex);
            // Obtain rewards for the action
            cumulatedReward += reward;
            // Check the cumulated rewards
            if (highestReward < cumulatedReward) {
                highestReward = cumulatedReward;
            }
            // Learn the state, action rewards pair
            qLearning.robotLearn(stateIndex, chosenAction,
                                 reward, nextStateIndex);
            // Quit generation if collision occurs
            if (collision) {
                // Reset the environment if collision occurs
                environmentReset();
                break;
            } else {
                stateIndex = nextStateIndex;
            }
            innerLoopCount++;
            ros::spinOnce();
        }
        ROS_INFO_STREAM("Epsilon: " << qLearning.getEpsilon()
                                    << " Episode Number: " << episodeCount
                                    << " Cum. reward: " << cumulatedReward);
        episodeCount++;
    } else {
        // Store qTable if training is complete
        if (!stored) {
            qLearning.setQtable(path);
            // Set true if training done and model stored
            stored = true;
        }
    }

    ros::spinOnce();
}

int Navigation::getStateIndex(std::vector<int> state) {
    int tempIndex = 0;
    // Set the initial states
    int tempState[4] = {0, 0, 0, 0};
    for (auto &&i : state) {
        // Update the states
        tempState[tempIndex++] += i - 1;
    }
    return (tempState[3] + tempState[2] * 6 + tempState[1] * 36 +
            (tempState[0]) * 216);
}

void Navigation::action(int action, bool &colStat, int &reward,
                                                               int &nextState) {
    // Set of action that can be taken while training
    std::vector<int> tempState;
    // Set liear motion for action = 0
    if (action == 0) {
        twistMessage.linear.x = 0.2;
        twistMessage.angular.z = 0.0;
        // publish the velocities
        velocityPublisher.publish(twistMessage);
    } else if (action == 1) {
        // If action = 1, turn right
        twistMessage.linear.x = 0.05;
        twistMessage.angular.z = 0.3;
        velocityPublisher.publish(twistMessage);
    } else if (action == 2) {
        // If action = 2 , turn left
        twistMessage.linear.x = 0.05;
        twistMessage.angular.z = -0.3;
        velocityPublisher.publish(twistMessage);
    }
    sensor_msgs::LaserScan pc;
    // Wait for the sensor data
    sensor_msgs::LaserScanConstPtr msg1 = ros::topic::waitForMessage
                            <sensor_msgs::LaserScan>("/scan", ros::Duration(1));
    if (msg1 == NULL)
        ROS_INFO_STREAM("Waiting for laser scan data");
    // Obtain the turtlebot states
    tempState = turtlebotStates.returnLaserState();
    // Obtain the state index for the states
    nextState = getStateIndex(tempState);
    // Obtain the colision status
    colStat = turtlebotStates.flagCollision();
    // If moving straight without collision, give higher rewards
    // If moving left or right without collsion give medium rewards
    if (colStat == false) {
        if (action == 0) {
            reward = 5;
        } else {
            reward = 1;
        }
    // If collision occurs, give punishment
    } else {
        reward = -200;
    }
}

void Navigation::environmentReset() {
    std_srvs::Empty resetWorldService;
    // Reset the gazebo world
    ros::service::call("/gazebo/reset_world", resetWorldService);
    // wait until the reseted environment is completely loaded
    while (turtlebotStates.flagCollision()) {
        ros::spinOnce();
    }
}

void Navigation::testRobot(double ix, double fx, double fy,
                            QLearning &qLearning, std::vector<int> state,
                            ros::Rate loop_rate) {
    // To determine delta x
    double inc_x = x_goal - x;
    // To detremine delta y
    double inc_y = y_goal - y;
    // to determine the angle of the goal from the robot's current orientation
    double angle_to_goal = atan2(inc_y, inc_x);
    // Check of goal is reached
    if (sqrt((x_goal - x) * (x_goal - x) + (y_goal - y) * (y_goal - y)) < 0.5) {
        twistMessage.linear.x = 0.0;
        twistMessage.angular.z = 0.0;
        // Check if the goal is first or second
        if (x_goal == ix) {
            ROS_INFO_STREAM("********************************************");
            ROS_INFO_STREAM("Reached workstation A on the assembly floor");
            ROS_INFO_STREAM("Put the PAYLOAD on the turtle bot.");
            for (int i : boost::irange(0, 50))
                loop_rate.sleep();
            ROS_INFO_STREAM("Ready DELIVER PAYLOAD to workstation B.");
            ROS_INFO_STREAM("********************************************");
        }
        x_goal = fx;
        y_goal = fy;
    } else {
        int stateIndex = 0;
        int chosenAction;
        // Set the epsilon
        qLearning.setEpsilon(-1);
        // Ontain the sensor states
        state = turtlebotStates.returnLaserState();
        stateIndex = getStateIndex(state);
        // Get next action based on current state and RL Model
        chosenAction = qLearning.demo(stateIndex,
                                      turtlebotStates.flagCollision(),
                                      abs(angle_to_goal - yaw));
        // Make the turtlebot perform the action
        demoAction(chosenAction);
        ros::spinOnce();
    }
}

void Navigation::demoAction(int action) {
    // Set of actions can be perfomed while testing robot
    if (action == 0) {
        // Set linear motion
        twistMessage.linear.x = 0.5;
        twistMessage.angular.z = 0.0;
        // Publish velocity
        velocityPublisher.publish(twistMessage);
    } else if (action == 1) {
        // set linear motion
        twistMessage.linear.x = 0;
        twistMessage.angular.z = 2;
        // Publish velocity
        velocityPublisher.publish(twistMessage);
    } else if (action == 2) {
        // Set linear motion
        twistMessage.linear.x = 0;
        twistMessage.angular.z = -3;
        // Publish velocity
        velocityPublisher.publish(twistMessage);
    }
    // Wait for LaserScan data for next iteration
    sensor_msgs::LaserScan pc;
    sensor_msgs::LaserScanConstPtr msg1 = ros::topic::waitForMessage
                                            <sensor_msgs::LaserScan>
                                            ("/scan", ros::Duration(10));
    if (msg1 == NULL) {
        ROS_ERROR("Waiting for laser scan data");
    }
}
