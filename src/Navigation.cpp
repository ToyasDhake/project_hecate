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
#include "Navigation.hpp"
#include <tf/transform_listener.h>

Navigation::Navigation() {
    velocityPublisher = nodeHandle.advertise <geometry_msgs::Twist>
 ("/mobile_base/commands/velocity", 1000);
    subscriber = nodeHandle.subscribe <sensor_msgs::LaserScan>
 ("/scan", 50, &TurtlebotStates::callDepth, &turtlebotStates);
    reward = 0;
    isCollision = false;
    twistMessage.linear.x = 0.0;
    twistMessage.linear.y = 0.0;
    twistMessage.linear.z = 0.0;
    twistMessage.angular.x = 0.0;
    twistMessage.angular.y = 0.0;
    twistMessage.angular.z = 0.0;
    x_goal = 4;
    y_goal = 5;
    velocityPublisher.publish(twistMessage);
}

Navigation::~Navigation() {
    velocityPublisher.publish(twistMessage);
    twistMessage.linear.x = 0.0;
    twistMessage.linear.y = 0.0;
    twistMessage.linear.z = 0.0;
    twistMessage.angular.x = 0.0;
    twistMessage.angular.y = 0.0;
    twistMessage.angular.z = 0.0;
}

void Navigation::trainRobot(std::string path) {
  int highestReward = 0;
  int stateIndex = 0;
  int episodeCount = 0;
  int totalEpisode = 100;
  double epsilonDiscount = 0.99;
  std::vector<int> state;
  int nextStateIndex;
  int chosenAction;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
  if (episodeCount < totalEpisode) {
      bool collision = false;
      int cumulatedReward = 0;
      environmentReset();
      if (qLearning.getEpsilon() > 0.05) {
          qLearning.setEpsilon(qLearning.getEpsilon() * epsilonDiscount);
      }

      state = turtlebotStates.returnLaserState();
      stateIndex = getStateIndex(state);

      int innerLoopCount = 0;

      while (innerLoopCount < 700) {
          chosenAction = qLearning.chooseAction(stateIndex);
          action(chosenAction, collision, reward, nextStateIndex);
          cumulatedReward += reward;

          if (highestReward < cumulatedReward) {
              highestReward = cumulatedReward;
          }
          qLearning.robotLearn(stateIndex, chosenAction, reward, nextStateIndex);
          if (collision) {
              environmentReset();
              break;
          } else {
              stateIndex = nextStateIndex;
          }
      innerLoopCount++;
      ros::spinOnce();
      }
      ROS_INFO_STREAM("Epsilon: " << qLearning.getEpsilon() << " Episode Number: " << episodeCount << " Cum. reward: " << cumulatedReward);
          episodeCount++;
     }
      ros::spinOnce();
  }
  ROS_INFO_STREAM("done");
  qLearning.setQtable(path);
}


void Navigation::dom(const nav_msgs::Odometry::ConstPtr& msg) {
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  z = msg->pose.pose.position.z;
  //double quatx= msg->pose.pose.orientation.x;
  //double quaty= msg->pose.pose.orientation.y;
  //double quatz= msg->pose.pose.orientation.z;
  // double quatw= msg->pose.pose.orientation.w;

  ///tf::Quaternion q(quatx, quaty, quatz, quatw);
  //tf::Matrix3x3 m(q);
  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  ROS_INFO_STREAM(x);

}

void Navigation::testRobot(std::string path) {
    std::vector<int> state;
    ros::Rate loop_rate(10);
    QLearning qLearning;
    qLearning.getQtable(path);
    environmentReset();
    ros::Subscriber sub_odom = nh.subscribe("odom", 10, &Navigation::dom, this);
    while (ros::ok()) {
        double inc_x = x_goal - x;
        double inc_y = y_goal - y;
        double angle_to_goal = atan2(inc_y,inc_x);
        int stateIndex = 0;
        int chosenAction;
        qLearning.setEpsilon(-1);
        state = turtlebotStates.returnLaserState();
        stateIndex = getStateIndex(state);
        chosenAction = qLearning.demo(stateIndex);
        ROS_INFO_STREAM( x);
        ROS_ERROR_STREAM(inc_x);
        ROS_ERROR_STREAM(inc_y);
        if (abs(inc_x) > 0.05 || abs(inc_y) > 0.05) {
            environmentUnpause();
            if (abs(angle_to_goal - yaw) > 0.05) {
                
                twistMessage.linear.x = 0;
                twistMessage.angular.z = 3;   
                velocityPublisher.publish(twistMessage);   
            }
            environmentPause();
            demoAction(chosenAction);
        }
        else {
            environmentUnpause();
                ROS_INFO_STREAM("Reached");
                twistMessage.linear.x = 0;
                twistMessage.angular.z = 0;   
                velocityPublisher.publish(twistMessage);   
            environmentPause();
        }
        
        if (turtlebotStates.isCollision()) {
            environmentReset();
        }
        ros::spinOnce();
    }
}

void Navigation::environmentReset() {

    std_srvs::Empty resetWorldService;
    ros::service::call("/gazebo/reset_world", resetWorldService);
    environmentUnpause();
    while (turtlebotStates.isCollision()) {
     ros::spinOnce();
    }
    environmentPause();
}

void Navigation::environmentPause() {
    std_srvs::Empty resetWorldService;
    ros::service::call("/gazebo/pause_physics", resetWorldService);
}

void Navigation::environmentUnpause() {
    std_srvs::Empty resetWorldService;
    ros::service::call("/gazebo/unpause_physics", resetWorldService);
}

void Navigation::action(int action, bool &colStat, int &reward, int &nextState) {
    std::vector<int> tempState;
    environmentUnpause();
    if (action == 0) {
        twistMessage.linear.x = 0.2;
        twistMessage.angular.z = 0.0;
        velocityPublisher.publish(twistMessage);
    } else if (action == 1) {
        twistMessage.linear.x = 0.05;
        twistMessage.angular.z = 0.3;
        velocityPublisher.publish(twistMessage);
    } else if (action == 2) {
        twistMessage.linear.x = 0.05;
        twistMessage.angular.z = -0.3;
        velocityPublisher.publish(twistMessage);
    }
    sensor_msgs::LaserScan pc;
    sensor_msgs::LaserScanConstPtr msg1 = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", ros::Duration(10));
    if (msg1 == NULL)
    ROS_ERROR("Waiting for laser scan data");
    environmentPause();
    tempState = turtlebotStates.returnLaserState();
    nextState = getStateIndex(tempState);
    colStat = turtlebotStates.isCollision();
    if (colStat == false) {
        if (action == 0) {
            reward = 5;
        } else {
            reward = 1;
        }
    } else {
        reward = -200;
    }
}

void Navigation::demoAction(int action) {
    environmentUnpause();
    if (action == 0) {
        twistMessage.linear.x = 0.2;
        twistMessage.angular.z = 0.0;
        velocityPublisher.publish(twistMessage);
    } else if (action == 1) {
        twistMessage.linear.x = 0.05;
        twistMessage.angular.z = 0.3;
        velocityPublisher.publish(twistMessage);
    } else if (action == 2) {
        twistMessage.linear.x = 0.05;
        twistMessage.angular.z = -0.3;
        velocityPublisher.publish(twistMessage);
    }
    sensor_msgs::LaserScan pc;
    sensor_msgs::LaserScanConstPtr msg1 = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", ros::Duration(10));
    if (msg1 == NULL) {
        ROS_ERROR("Waiting for laser scan data");
    }
    environmentPause();
}

int Navigation::getStateIndex(std::vector<int> state) {
    int tempIndex = 0;
    int tempState[4] = {0, 0, 0, 0};
    for (auto&& i : state) {
        tempState[tempIndex++] += i-1;
    }
    return (tempState[3] + tempState[2] * 6 + tempState[1] * 36 + (tempState[0])*216);
}
