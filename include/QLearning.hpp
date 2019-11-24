#ifndef INCLUDE_QLEARNING_HPP_
#define INCLUDE_QLEARNING_HPP_


#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <array>

class QLearning {
 private:
     std::vector <std::vector<double>> qTable;
     double epsilon = 0.9;
     double alpha = 0.2;
     double gamma = 0.8;
     
 public: 
     QLearning();
     void setEpsilon(double e);
     double getEpsilon();
     void setQtable(std::string path);
     void getQtable(std::string path);
     void qLearn(int state, int action, int reward, double val);
     void robotLearn(int si, int act, int rew, int nsi);
     void testStoreQ();
     int demo(int index);
     int chooseAction(int index);
     
};

#endif // INCLUDE_QLEARNING_HPP_