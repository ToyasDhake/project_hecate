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
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  add_rostest_gtest(
  hecateTest
  test/rltest.launch
  test/main.cpp
  test/NavigationTest.cpp
  test/QlearningTest.cpp
  test/TurtlebotstatesTest.cpp
  src/Navigation.cpp
  src/QLearning.cpp
  src/TurtlebotStates.cpp
  )
  target_link_libraries(
  hecateTest
  ${catkin_LIBRARIES}
  )
  add_dependencies(
  hecateTest
  hecate
  ${catkin_EXPORTED_TARGETS}
  )
endif()
