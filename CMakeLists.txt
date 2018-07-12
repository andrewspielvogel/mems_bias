cmake_minimum_required(VERSION 2.8.3)
project(mems_bias)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  dscl_msgs
  rospy
)


find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} dscl_msgs
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs roscpp ${MSG_DEPS}
)



include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})



add_executable(mems_bias_publisher src/mems_bias_publisher.cpp src/mems_bias.cpp)
target_link_libraries(mems_bias_publisher ${catkin_LIBRARIES})
set_target_properties(mems_bias_publisher PROPERTIES COMPILE_FLAGS "-O3")

