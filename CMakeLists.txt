cmake_minimum_required(VERSION 2.8.3)
project(mems_bias)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  genmsg
  rospy
)

add_message_files(
  FILES
  Imu9DOF.msg
  ImuBias.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mems_bias
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs roscpp
)

## For Eigen library support
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include_directories(include ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})



add_executable(mems_bias_publisher src/mems_bias_publisher.cpp src/mems_bias.cpp)
target_link_libraries(mems_bias_publisher ${catkin_LIBRARIES})
set_target_properties(mems_bias_publisher PROPERTIES COMPILE_FLAGS "-O3")

