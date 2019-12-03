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
* @file QLearning.hpp
* @author Shivam Akhauri, Toyas Dhake
* @date 27 November 2019
* @copyright BSD 3-clause, 2019 Shivam Akhauri,Toyas Dhake 
* @brief Header for the RL algorithm implementation
*/
#ifndef _HOME_TOYAS_CATKIN_WS_SRC_PROJECT_HECATE_INCLUDE_QLEARNING_HPP_
#define _HOME_TOYAS_CATKIN_WS_SRC_PROJECT_HECATE_INCLUDE_QLEARNING_HPP_

#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <array>

/**
 * @brief Class Qlearning
 * class to perform reinforcement learning algorithm
 */
 
class QLearning {
 private:
    std::vector<std::vector<double>> qTable;
    double epsilon = 0.9;
    double alpha = 0.2;
    double gamma = 0.8;
    int pauseGoal = 0;
    bool qTableGood = false;

 public:
    /**
    * @brief constructor Qlearning class
    * @param none
    * @return none
    * intililizes the reinforcement learning model
    */
    QLearning();
    /**
    * @brief function setEpsilon
    * @param double e
    * @return none
    * setter for epsilon
    */
    void setEpsilon(double e);
    /**
    * @brief function getEpsilon
    * @param none
    * @return double epsilon as
    * getter for epsilon
    */
    double getEpsilon();
    /**
    * @brief function setQtable
    * @param std::string path 
    * @return none
    * stores the rl model
    */
    void setQtable(std::string path);
    /**
    * @brief function getQtable
    * @param std::string path 
    * @return none
    * loads the pretrained RL model
    */
    void getQtable(std::string path);
    /**
    * @brief function qlearn
    * @param int state 
    * @param  int action 
    * @param  int reward
    * @param  double val
    * @return none
    * updates reinforcement learning model
    */
    double qLearn(int state, int action, int reward, double val);
     /**
    * @brief function robotLearn
    * @param int si
    * @param int act 
    * @param int rew 
    * @param int nsi
    * @return none
    * applies the boltzmann equation to apply RL
    */
    int robotLearn(int si, int act, int rew, int nsi);
    /**
    * @brief function testStoreQ
    * @param none
    * @return none
    * function for inference quality test of the rl model
    */
    void testStoreQ();
    /**
    * @brief function demo
    * @param int index 
    * @return int action 
    * use the rl model to decide the best action
    */
    int demo(int index, bool collision, double angleToGoal);
     /**
    * @brief function chooseAction
    * @param  int index 
    * @return int action 
    * robots action selection for the state
    */
    int chooseAction(int index);
};

#endif  // _HOME_TOYAS_CATKIN_WS_SRC_PROJECT_HECATE_INCLUDE_QLEARNING_HPP_
