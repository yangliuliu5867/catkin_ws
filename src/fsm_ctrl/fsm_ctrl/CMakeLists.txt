cmake_minimum_required(VERSION 3.0.2)
project(fsm_ctrl)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  mavros_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  tf
  quadrotor_msgs
  ctrl_math
)
find_package(Eigen3 REQUIRED)
find_package(casadi CONFIG QUIET)
if(NOT TARGET casadi::casadi)
  find_path(CASADI_INCLUDE_DIR NAMES casadi/casadi.hpp PATHS /usr/local/include)
  find_library(CASADI_LIBRARY NAMES casadi PATHS /usr/local/lib)
  if(NOT CASADI_INCLUDE_DIR OR NOT CASADI_LIBRARY)
    message(FATAL_ERROR "CasADi was not found. Install it or set casadi_DIR.")
  endif()
  add_library(casadi::casadi UNKNOWN IMPORTED)
  set_target_properties(casadi::casadi PROPERTIES
    IMPORTED_LOCATION "${CASADI_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${CASADI_INCLUDE_DIR}")
endif()

add_message_files(
  FILES
  nmpc_state.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  mavros_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs quadrotor_msgs roscpp sensor_msgs std_msgs tf message_runtime ctrl_math
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(single_offboard_fsm
  src/single_offboard_fsm.cpp
  src/NMPC_Controller.cpp
)
target_link_libraries(single_offboard_fsm
  ${catkin_LIBRARIES}
  casadi::casadi
)
add_dependencies(single_offboard_fsm
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(px4_estimator
  src/px4_estimator.cpp
)
target_link_libraries(px4_estimator
  ${catkin_LIBRARIES}
)

add_executable(swarm_user_cmd
  src/swarm_user_cmd.cpp
)
target_link_libraries(swarm_user_cmd
  ${catkin_LIBRARIES}
)
add_dependencies(swarm_user_cmd
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
