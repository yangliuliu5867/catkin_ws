cmake_minimum_required(VERSION 3.0.2)
project(real_flight_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  quadrotor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/publish_goal_once.py
  scripts/publish_reference_start_once.py
  scripts/wrench_to_force_vector.py
  scripts/controller_stability_test.py
  scripts/replay_position_command.py
  scripts/generate_static_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
