[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/ToyasDhake/project_hecate/blob/master/License) [![Build Status](https://travis-ci.org/ToyasDhake/project_hecate.svg?branch=master)](https://travis-ci.org/ToyasDhake/project_hecate) [![Coverage Status](https://coveralls.io/repos/github/ToyasDhake/project_hecate/badge.svg?branch=master)](https://coveralls.io/github/ToyasDhake/project_hecate?branch=master)


# project_hecate

## Overview 

We propose a Self navigating package delivery robot, capable of finding route between logistic stations and deliver mobile parts like electric circuits, motherboards, screens and similar embedded parts from the manufacturing unit to the assembly line, in large factory units, like the Apple’s factory in China. Such autonomous robotic system with inherent artificial intelligence to find it’s way in factories and avoid collisions while traversing, has been developed to yield big returns to Acme robotics.

## Main features of the product
- Capable of ‘learning to find it’s way’ in a factory/random environment
- Obstacle avoidance
- Stays at its default location (spawns at origin in the gazebo world) and when user commands to deliver a package, it moves to Point A to collect the package. It waits for the factory worker to put the package on it for 5 seconds and then moves towards the Point B, to deliver the package.
- Autonomous navigation

## System Design and Algorithm


## Demo Steps

### Build Steps
```
cd mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/ToyasDhake/project_hecate.git
cd ..
catkin_make
```

### Demo Steps

The user has to specify two points in the gazebo world-
1. Point A- This is the point the turtleboit navigates to, from the origin resting place,in order to receive the load package from the factory worker. The turtlebot waits for the factory worker for about 5 seconds to put on the load. (syntax: xInitial:= X Coordinate of Point A  yInitial:= Y Coordinate of Point A)
2. Point B- This is the point the turtlebot navigates to, after picking up the load from Point A, to drop the load at Point B.
(syntax: xFinal:= X Coordinate of Point B  yFinal:= Y Coordinate of Point B
For example, in the commands below, Point A coordinates is (2,2) and Point B coordinates is (0,7). With our experiments we found that this is one of the tough combinations for the RL to predict trajectory of the turtlebot, but our results are pretty good even on these points.

```
#To load Default RL trained model
roslaunch project_hecate testHecate.launch xInitial:=2 yInitial:=2 xFinal:=0 yFinal:=7

# To train a custom model
roslaunch project_hecate trainHecate.launch path:=<path to save>

#To load custom RL model trained by the user
roslaunch project_hecate testHecate.launch xInitial:=2 yInitial:=2 xFinal:=0 yFinal:=7 path:=<path to table>
```

### Test Steps
```
cd ~/catkin_ws/
catkin_make run_tests
rostest project_hecate rltest.launch
```

### Doxygen Steps
```
sudo apt install doxygen
cd <project_hecate repo>
doxygen -g
doxygen
cd latex
make

```

## Dependencies
ROS Kinetic

TurtleBot v2

ROS Kinetic

Gazebo 7.4 and above

Catkin



## Results
The following video shows the training of the turtlebot to "learn its way" through a floor map. During training, the turtlebot starts from the origin and then tries to navigate by taking actions of - going straight, take a left or take a right, in each episode. For each of these three actions, the turtlebot receives a reward. An episode involves a set of actions till the turtlebot collides. The episode ends after collision. The priciple during training is to achieve maximum sum of rewards in an episode. With more epochs of training, the turtlebot tries to maximize its rewards in the episode and stores the actions it took for the given states, which led to it earning maximum episode rewards.

During Inference, the turtlebot uses its learnt knowledge during training to decide on what actions to take, given a state, which had earlier led to it earn maximum rewards during training.


## Assumptions: 
-We assume that the gazebo world is not changed drastically. Although the RL algorithm is capable of performing well in a dynamic world it was not trained on, drastic changes may require hyperparameter tuning of the algorithm.
-We train the model on the gazebo simulator and assume that it performs well on real world too.
-Acme Robotics has powerful systems with Ubuntu 16 and Ros kinetics with Gazebo (I7 processor, 16 GB RAM).
-We assume that the obstacles are stationary.

## Documentation

### Product Backlog and Sprint Schedule
The product backlog file can be accessed at:
https://docs.google.com/spreadsheets/d/1CMIzxtqc-AxdCg9Mqs4tmX4eBPp3Yyy5vdFZ9n3fnpU/edit?usp=sharing

The Sprint planning and review document can be accessed at:
https://docs.google.com/document/d/1bXLFW7gJ9vdtRvNPkyLKW2za1OYg1eaJVJBhiPVOmLE/edit?usp=sharing

The presentation is available at:
https://umd0-my.sharepoint.com/:p:/r/personal/sakhauri_umd_edu/Documents/Presentation.pptx?d=wbe14eef608b648c7bbd99860123441db&csf=1&e=8Ihqd1

## Known Issues and Limitations
1. The RL algorithm is under active research. The algorithm implemented navigates the robot autonomously and collision free from point A to Point B, but ocassionally the path taken is not highly optimized. 
2. The training of the turtlebot is highly compute intensive.
3. The Reinforcement learning algorithm was developed with hyperparametrs optimized for the gazebo world used in the simulation. New worlds may requires training the RL world with hyperparameter tuning and modifications.
4. The user has to define the Point A and Point B within the rectangular walls of the gazebo world. If not done so, the turtlebot would go towards the wall to reach the point, then avoid it and go back again and repeat in a loop.
5 The Cpplint forbids the use of "non-const reference". But passing "const" to ROS function callbacks is not allowed.  


## Developer Documentation
1. To train the model on a new gazebo world, tune the hyperparamers like the epsilon value, rewards. The developer might have to experiment with the linear and angular velocities for the robot to move take actions slower for the Rl states for better training.
2. Train the model with good number of epochs.
3. Create the walls and other objects in gazebo in the form of gazebo models so that after each epoch of training, when we reset the environment, the objects do not align back to their original orientations. 

## License
```
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
```
## Contributors

### Shivam Akhauri 

Former Artificial Intelligence Engineer at Ether Labs. -Former Machine learning Engineer and Project Lead at Tata Elxsi. -Skilled in AI/ML with applications in Computer vision, NLP and Robotics.


### Toyas Dhake

Robotics engineer, University of Maryland College Park. -Skilled in embedded system with applications involving Arduino, Raspberry Pi and Jetson Boards.

