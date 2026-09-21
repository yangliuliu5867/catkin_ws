cmake_minimum_required(VERSION 3.0.2)
project(ctrl_math)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ctrl_math
  CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(ctrl_math
  src/ctrl_math.cpp
)
target_link_libraries(ctrl_math
  ${catkin_LIBRARIES}
)
