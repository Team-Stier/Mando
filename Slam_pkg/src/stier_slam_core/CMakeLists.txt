cmake_minimum_required(VERSION 3.0.2)
project(stier_slam_core)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  message_filters
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  rosbag
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    message_filters
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
    tf2_ros
    rosbag
)

catkin_add_nosetests(test)

catkin_install_python(PROGRAMS
  scripts/wheel_encoder_adapter_node.py
  scripts/vehicle_odometry_node.py
  scripts/scan_leveler_node.py
  scripts/rtk_gate_node.py
  scripts/map_release_cli.py
  scripts/live_obstacle_node.py
  scripts/sensor_health_node.py
  scripts/rtabmap_runtime_guard.py
  scripts/map_editor_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY web/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/web
)
