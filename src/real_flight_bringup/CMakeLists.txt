cmake_minimum_required(VERSION 3.0.2)
project(real_flight_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  quadrotor_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(lidar_imu_state_bridge src/lidar_imu_state_bridge.cpp)
add_dependencies(lidar_imu_state_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_imu_state_bridge ${catkin_LIBRARIES})

add_executable(polytraj_poscmd_bridge_node src/polytraj_poscmd_bridge_node.cpp)
add_dependencies(polytraj_poscmd_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(polytraj_poscmd_bridge_node ${catkin_LIBRARIES})

add_executable(collision_recovery_nmpc_node src/collision_recovery_nmpc_node.cpp)
add_dependencies(collision_recovery_nmpc_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_recovery_nmpc_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/publish_goal_once.py
  scripts/publish_reference_start_once.py
  scripts/wrench_to_force_vector.py
  scripts/controller_stability_test.py
  scripts/replay_position_command.py
  scripts/generate_static_map.py
  scripts/thrust_calibrate_nmpc.py
  scripts/rviz_goal_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS lidar_imu_state_bridge polytraj_poscmd_bridge_node collision_recovery_nmpc_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
