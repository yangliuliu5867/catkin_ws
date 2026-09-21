cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  uav_utils
  mavros
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)
add_dependencies(px4ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4ctrl_node ${catkin_LIBRARIES})

add_executable(px4traj_node src/px4traj_node.cpp)
add_dependencies(px4traj_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4traj_node ${catkin_LIBRARIES})

add_executable(polytraj_poscmd_bridge_node src/polytraj_poscmd_bridge_node.cpp)
add_dependencies(polytraj_poscmd_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(polytraj_poscmd_bridge_node ${catkin_LIBRARIES})

add_executable(admittance_control_node src/admittance_control_node.cpp)
add_dependencies(admittance_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(admittance_control_node ${catkin_LIBRARIES})

add_executable(collision_recovery_node src/collision_recovery_node.cpp)
add_dependencies(collision_recovery_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_recovery_node ${catkin_LIBRARIES})

add_executable(poscmd_admittance_overlay_node src/poscmd_admittance_overlay_node.cpp)
add_dependencies(poscmd_admittance_overlay_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(poscmd_admittance_overlay_node ${catkin_LIBRARIES})

# add_executable(test_attitude_target_node src/test_attitude_target.cpp)
# target_link_libraries(test_attitude_target_node ${catkin_LIBRARIES})

# add_executable(debug_node src/debug_node.cpp)
# target_link_libraries(debug_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
