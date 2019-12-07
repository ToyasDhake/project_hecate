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
* @file TurtlebotStates.hpp
* @author Shivam Akhauri, Toyas Dhake
* @date 27 November 2019
* @copyright BSD 3-clause, 2019 Shivam Akhauri,Toyas Dhake 
* @brief Header for reading the robot current states
**/
#ifndef INCLUDE_TURTLEBOTSTATES_HPP_
#define INCLUDE_TURTLEBOTSTATES_HPP_

#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

/**
 * @brief Class depthCalculatio
 * This class contains members to calculate distance 
 * for the objects which is obtained from laserscan topic.
 * It also contains members to raise a flag if about to collide
*/

class TurtlebotStates {
 private:
    // raise flag if very close to the obstacle
    bool collisionStatus = false;
    double previousNearest = 10;
    std::vector<int> laserState;

 public:
    /**
    * @brief constructor TurtlebotStates
    * @param none
    * @return none
    * initializes the collisionStatus flag
    */
    TurtlebotStates();

    /**
    * @brief destructor TurtlebotStates
    * @param none
    * @return none
    * destroy the TurtlebotStates
    */
    ~TurtlebotStates();

    /**
    * @brief function findLaserDepth
    * @param msg type sensor_msgs::LaserScan
    * @return none 
    * function to read LaserScan sensor messages and raise flag 
    * if distance of the obstacle is less than threshold 
    */
    void findLaserDepth(const sensor_msgs::LaserScan::ConstPtr &msg);

    /**
    * @brief function flagCollision
    * @param none
    * @return 1 if very close to obstacle and 0 if not close
    * Return the current value of collisionStatus
    */
    bool flagCollision();
    std::vector<int> returnLaserState();
};

#endif  // INCLUDE_TURTLEBOTSTATES_HPP_
