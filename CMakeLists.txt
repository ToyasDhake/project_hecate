cmake_minimum_required(VERSION 2.8.3)
project(project_hecate)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs move_base_msgs sensor_msgs geometry_msgs genmsg std_srvs)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(qlearn src/main.cpp src/TurtlebotStates.cpp src/Navigation.cpp src/QLearning.cpp)
target_link_libraries(qlearn ${catkin_LIBRARIES})
