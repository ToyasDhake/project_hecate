#ifndef INCLUDE_NAVIGATION_HPP_
#define INCLUDE_NAVIGATION_HPP_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "TurtlebotStates.hpp"
#include "QLearning.hpp"
 
class Navigation {
 private:
    QLearning qLearning;
    ros::NodeHandle nodeHandle;
    ros::Publisher velocityPublisher;
    ros::Subscriber subscriber;
    geometry_msgs::Twist twistMessage;
    TurtlebotStates turtlebotStates;
    int reward;
    bool isCollision;

 public:
    Navigation();
    ~Navigation();
    void trainRobot(std::string path);
    void testRobot(std::string path);
    void environmentReset();
    void environmentPause();
    void environmentUnpause();
    void action(int action, bool &colStatus, int &reward, int &nextState);
    void demoAction(int action);
    int getStateIndex(std::vector<int> state);
};

#endif // INCLUDE_NAVIGATION_HPP_
