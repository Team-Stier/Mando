cmake_minimum_required(VERSION 3.0.2)
project(stier_slam_test_support)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  rosgraph_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_package(CATKIN_DEPENDS
  diagnostic_msgs
  geometry_msgs
  rosgraph_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/synthetic_sensor_node.py
  test/test_synthetic_topics.py
  test/test_synthetic_wall_time.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/synthetic_topics.test)
  add_rostest(test/synthetic_wall_time.test)
endif()
