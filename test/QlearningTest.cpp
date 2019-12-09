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
 * @brief Test to verify expected turtlebot action
 * @param      TESTQlearning    gtest framework
 * @param      testChooseAction   Name of the test
 * @return     none
 */
TEST(TESTQlearning, testChooseAction) {
    // Constructor
    QLearning q;
    // Use the trained model to predict action
    q.testStoreQ();
    // set epsilon
    q.setEpsilon(0);
    int epsi = q.getEpsilon();
    // return expected action
    ASSERT_EQ(2, q.chooseAction(epsi));
}

/**
 * @brief Test to verify if demo is taking proper decision 
 *        based on values in table
 * @param      TESTQlearning    gtest framework
 * @param      testActionfromTheDemoFunctions   Name of the test
 * @return     none
 */
TEST(TestQlearning, testActionfromTheDemoFunctions) {
    // constructor
    QLearning q;
    // load the trained RL model
    q.testStoreQ();
    // test the demo function
    int action = q.demo(0, 1, 0.5);
    ASSERT_NE(0, action);
}
