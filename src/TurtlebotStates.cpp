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
* @file TurtlebotStates.cpp
* @author Shivam Akhauri, Toyas Dhake
* @date 27 November 2019
* @copyright BSD 3-clause, 2019 Shivam Akhauri,Toyas Dhake
* @brief Code to read the current states of the turtlebot 
*/
#include <iostream>
#include <cfloat>
#include <cmath>
#include "TurtlebotStates.hpp"
#include <boost/range/irange.hpp>

TurtlebotStates::TurtlebotStates() {
    // set the collision flag as false by default
    collisionStatus = false;
}

TurtlebotStates::~TurtlebotStates() {
}

void TurtlebotStates::findLaserDepth(const sensor_msgs::LaserScan::ConstPtr
                                         &msg) {
    // Check if collision is going to occur based on laser scan data
    double nearest = 999;
    for (auto temp : msg->ranges) {
        if (temp < nearest)
            nearest = temp;
    }
    if (nearest == 999) {
        if (previousNearest < 3) {
            // Set the collision flag to be true if obstacle close
            collisionStatus = true;
        } else {
            // Set the collsion flag to be false if obstacle far
            collisionStatus = false;
        }
    } else {
        previousNearest = nearest;
        if (nearest < 0.8) {
            // If obstacle near, set the collsion status to true
            collisionStatus = true;
        } else {
            // If obstacle far, set the collision status to be false
            collisionStatus = false;
        }
    }
    // Pack laser scan data
    std::vector<int> tempLaserState;
    int mod = (msg->ranges.size() / 4);
    for (int i : boost::irange(0, static_cast<int>(msg->ranges.size()))) {
        if (i % mod == 0) {
            // load the sensor data in the buffer
            if (std::isnan(msg->ranges[i])) {
                tempLaserState.push_back(6);
            } else {
                tempLaserState.push_back(round(msg->ranges[i]));
            }
        }
    }
    laserState = tempLaserState;
    tempLaserState.clear();
}

bool TurtlebotStates::flagCollision() {
    // return obstacle status
    return collisionStatus;
}

std::vector<int> TurtlebotStates::returnLaserState() {
    // return Laser states
    return laserState;
}
