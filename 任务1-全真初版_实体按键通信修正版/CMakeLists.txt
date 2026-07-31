cmake_minimum_required(VERSION 3.0.2)
project(d26_air_ground_uav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/air_ground_mission_fsm.py
  scripts/drop_actuator_serial.py
  scripts/platform_vision_node.py
  scripts/uav_udp_gateway.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config docs esp32
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
