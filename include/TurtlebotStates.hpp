#ifndef INCLUDE_TURTLEBOTSTATES_HPP_
#define INCLUDE_TURTLEBOTSTATES_HPP_

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class TurtlebotStates {
 private:
  std::vector<int> laserState;
  bool collisionStatus = false;

 public:
  TurtlebotStates();
  ~TurtlebotStates();
  std::vector<int> returnLaserState();
  bool isCollision();
  void callDepth(const sensor_msgs::LaserScan::ConstPtr& msg);
};

#endif // INCLUDE_TURTLEBOTSTATES_HPP_