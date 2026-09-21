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
  quadrotor_msgs
  uav_utils
  px4ctrl
  tf
  visualization_msgs
  ctrl_math
)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compile the ORIGINAL catkin_ws control sources, without changing or copying them.
# This package retains the baseline src/fsm_ctrl/fsm_ctrl layout.
get_filename_component(PX4CTRL_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../px4ctrl" ABSOLUTE)
if(NOT EXISTS "${PX4CTRL_SOURCE_DIR}/src/controller.cpp")
  message(FATAL_ERROR "Expected original px4ctrl sources at ${PX4CTRL_SOURCE_DIR}")
endif()

generate_messages(
  DEPENDENCIES
  geometry_msgs
  mavros_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PX4CTRL_SOURCE_DIR}/src
)

add_executable(single_offboard_fsm
  src/single_offboard_fsm.cpp
  src/px4ctrl_hover.cpp
  ${PX4CTRL_SOURCE_DIR}/src/controller.cpp
  ${PX4CTRL_SOURCE_DIR}/src/PX4CtrlParam.cpp
  ${PX4CTRL_SOURCE_DIR}/src/input.cpp
)
add_dependencies(single_offboard_fsm ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_offboard_fsm
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
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
  ${CMAKE_THREAD_LIBS_INIT}
)

install(TARGETS single_offboard_fsm px4_estimator swarm_user_cmd
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(px4ctrl_hover_test tests/px4ctrl_hover.test
    tests/px4ctrl_hover_test.cpp src/px4ctrl_hover.cpp
    ${PX4CTRL_SOURCE_DIR}/src/controller.cpp
    ${PX4CTRL_SOURCE_DIR}/src/PX4CtrlParam.cpp
    ${PX4CTRL_SOURCE_DIR}/src/input.cpp)
  if(TARGET px4ctrl_hover_test)
    add_dependencies(px4ctrl_hover_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(px4ctrl_hover_test ${catkin_LIBRARIES})
  endif()
endif()
