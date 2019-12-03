cmake_minimum_required(VERSION 2.8.3)
project(project_hecate)

add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs move_base_msgs sensor_msgs geometry_msgs tf)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hecate src/main.cpp src/Navigation.cpp src/TurtlebotStates.cpp src/QLearning.cpp)
target_link_libraries(hecate ${catkin_LIBRARIES})


## Testing
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)

#  add_rostest_gtest(
#  tf_talker test/talker.launch test/talker.cpp)
#  target_link_libraries(tf_talker ${catkin_LIBRARIES})
#  add_dependencies(tf_talker talkWithParentWorld ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#endif()
