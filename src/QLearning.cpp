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
* @file QLearning.cpp
* @author Shivam Akhauri, Toyas Dhake
* @date 27 November 2019
* @copyright BSD 3-clause, 2019 Shivam Akhauri,Toyas Dhake
* @brief Code to define the reinforcement learning pipeline
*/
#include <time.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <cstdlib>
#include <random>
#include <boost/range/irange.hpp>
#include "QLearning.hpp"

QLearning::QLearning() {
    // Create a table
    for (int i : boost::irange(0, 1296)) {
        std::vector<double> row(3, 0.0);
        // Store the states
        qTable.push_back(row);
    }
}

void QLearning::setEpsilon(double e) {
    // Set epsilon
    epsilon = e;
}

double QLearning::getEpsilon() {
    // return epsilon
    return epsilon;
}

void QLearning::setQtable(std::string path) {
    // Store QTable genrated from training of robot
    ROS_INFO_STREAM(path);
    std::ofstream out(path);
    for (auto &&i : qTable) {
        for (auto &&j : i)
            out << j << ',';
        out << '\n';
    }
    ROS_INFO("Qtable Stored");
}

void QLearning::getQtable(std::string path) {
    // Load stored data for robot testing
    std::vector<double> temp;
    std::vector<std::vector<double>> temp2;
    std::ifstream file(path);
    std::string row, cell;
    // Read the model
    if (file.good()) {
        ROS_INFO("File loaded");
        int rowCount = 0;
        while (std::getline(file, row)) {
            int columnCount = 0;
            std::stringstream linestream(row);
            // Read the model
            while (getline(linestream, cell, ',')) {
                // Read cell wise
                std::stringstream convertor(cell);
                convertor >> qTable[rowCount][columnCount];
                // Read the next cell
                ++columnCount;
            }
            temp2.push_back(temp);
            ++rowCount;
        }
        qTableGood = true;
    }
}

void QLearning::qLearn(int state, int action, int reward, double val) {
    // Genrate enteries to be done to qtable based on current state and action
    double currentValue = qTable[state][action];
    if (currentValue == 0) {
        qTable[state][action] = reward;
    } else {
        qTable[state][action] = currentValue + alpha * (val - currentValue);
    }
}

void QLearning::robotLearn(int si, int act, int rew, int nsi) {
    // RL model logic
    std::vector<double> qNextState;
    qNextState = qTable[nsi];
    auto maxIterator = std::max_element(std::begin(qNextState),
                                                          std::end(qNextState));
    int maxIndex = std::distance(std::begin(qNextState), maxIterator);
    // Take the action with the maximum boltzamnn value
    auto largest = qNextState[maxIndex];
    // Clear states
    qNextState.clear();
    // Apply the boltzmann equation
    qLearn(si, act, rew, rew + gamma * largest);
}

void QLearning::testStoreQ() {
    qTable[0][0] = 1;
    qTable[0][1] = 2;
    qTable[0][2] = 3;
}

int QLearning::demo(int index, bool collision, double angleToGoal) {
    int maxIndex = 0;
    // Check if file is loaded
    if (qTableGood) {
        std::vector<double> qState;
        qState = qTable[index];
        // Obtain maximum value as per the boltzmann equation
        auto maxIterator = std::max_element(std::begin(qState),
                                            std::end(qState));
        maxIndex = std::distance(std::begin(qState), maxIterator);
        if (pauseGoal > 0)
            pauseGoal--;
        if (pauseGoal == 0) {
            if (angleToGoal > 0.1) {
                maxIndex = 2;
            }
        }
    } else {
        // Fail safe if file does not load
        if (collision) {
            // stop linear motion and begin angular velocity of
            // turtlebot to avoid collision
            maxIndex = 1;
            pauseGoal = 20;
        } else {
            // if no obstacle move straight
            maxIndex = 0;
            if (pauseGoal > 0)
                pauseGoal--;
            if (pauseGoal == 0) {
                if (angleToGoal > 0.1) {
                    maxIndex = 2;
                }
            }
        }
    }
    return maxIndex;
}

int QLearning::chooseAction(int index) {
    // Select action for next step based on current genration data
    std::vector<double> qState;
    qState = qTable[index];
    // Choose the maximum state value
    auto maxIterator = std::max_element(std::begin(qState), std::end(qState));
    int maxIndex = std::distance(std::begin(qState), maxIterator);
    // Choose the state index with maximum state index
    auto largest = qState[maxIndex];
    int selectedAction = 0;

    std::random_device rd;
    // Introduce some randomness in model
    std::mt19937 gen(rd());
    // Obtain the gaussian distribution for the epsilon value
    std::uniform_real_distribution<> dis(0.0, 1.0);
    float randNum = dis(gen);

    if (randNum < epsilon) {
        auto minIterator = std::min_element(std::begin(qState),
                                            std::end(qState));
        int minIndex = std::distance(std::begin(qState), minIterator);
        // Obtain the qstate value
        auto mag = qState[maxIndex];
        if ((std::fabs(qState[minIndex])) > (std::fabs(qState[maxIndex]))) {
            mag = qState[minIndex];
        }
        for (int i : boost::irange(0, 3)) {
            qState[i] += randNum * mag - 0.5 * mag;
        }
        maxIterator = std::max_element(std::begin(qState), std::end(qState));
        maxIndex = std::distance(std::begin(qState), maxIterator);
        // Return the state index with the largest value
        largest = qState[maxIndex];
    }

    std::vector<int> largestQ;
    for (int i : boost::irange(0, 3)) {
        // Return the state index with the largest value
        if (largest == qState[i])
            largestQ.emplace_back(i);
    }

    if (largestQ.size() > 1) {
        std::uniform_int_distribution<> disact(0, largestQ.size() - 1);
        selectedAction = largestQ[disact(gen)];
    } else {
        selectedAction = largestQ[0];
    }
    // Reset the states
    largestQ.clear();
    qState.clear();
    return selectedAction;
}
