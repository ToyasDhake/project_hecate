#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <time.h>
#include <cmath>
#include <cstdlib>
#include <random>
#include "../include/QLearning.hpp"

QLearning::QLearning() {
}

void QLearning::setEpsilon(double e) {
    epsilon = e;
}

double QLearning::getEpsilon() {
    return epsilon;
}

void QLearning::setQtable(std::string path) {
}

void QLearning::getQtable(std::string path) {
}

void QLearning::qLearn(int state, int action, int reward, double val) {
}

void QLearning::robotLearn(int si, int act, int rew, int nsi) {
}

void QLearning::testStoreQ() {
}

int QLearning::demo(int index) {
    return 0;
}

int QLearning::chooseAction(int index) {
    return 0;
}
