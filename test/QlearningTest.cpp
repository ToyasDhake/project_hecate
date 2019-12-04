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
 * @file    QlearningTest.cpp
 * @author  Shivam Akhauri, Toyas Dhake
 * @copyright 3-clause BSD
 * @brief Test cases for class Qlearning
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "QLearning.hpp"

/**
 * @brief RL algorithms uses Boltzmann equation to calculate 
 * the value reward for the robot for taking a particular action
 * This test verifies the value obtained from boltzmann equation
 */
TEST(TestQlearning1, verifyBoltzmannValueForQlearn) {
    QLearning rl;
    double reward = 5.0;
    int action = 4;
    int state = 1;
    bool checkInvalid = rl.qLearn(state, action,reward, 5.0);
    ASSERT_TRUE(checkInvalid);
}
/**
 * @brief Test to verify if correct actions 
 * are returned for the given state index and the collision flag
 * during training state
 */
TEST(TestQlearning1, testActionfromTheDemoFunctions) {
    QLearning q;
    q.testStoreQ();
    int action = q.demo(0,1,0.5);
    ASSERT_NE(0, action);
}
/**
 * @brief Test to verify if correct actions 
 * are returned for the given state index and collision flag
 * during the training state
 */
TEST(TESTQlearning, testChooseAction) {
    QLearning q;
    q.testStoreQ();
    q.setEpsilon(-1);
    ASSERT_EQ(2, q.chooseAction(0));
}

/**
 * @brief Test to verify if the q learning 
 * class object is being initialized correctly
 */
TEST(TESTQlearning, testIntializationError) {
    EXPECT_NO_FATAL_FAILURE(QLearning q);
}

/**
 * @brief RL algorithms uses Boltzmann equation to calculate 
 * the value reward for the robot for taking a particular action
 * This test verifies that correct state index is mapped with 
 * the correct action predicted by the boltzmann equation
 */
TEST(TestQlearning1, verifyMaxBoltzmannIndex) {
    QLearning rl;
    int reward = 5.0;
    int action = 4;
    int stateIndex = 1;
    ASSERT_EQ(0,rl.robotLearn(stateIndex, action, reward, 1));
}

/**
  * @brief Test to verify if correct actions 
 *  are mapped to correct states based on the model 
 *  trained during the inference 
 */
TEST(TestQlearning1, verifyActionMappingToIndex) {
    QLearning rl;
    int index = 1;
    ASSERT_EQ(1,rl.chooseAction(index));
}




