cmake_minimum_required(VERSION 3.0.2)
project(stier_slam_bringup)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  test/test_launch_contracts.py
  test/test_synthetic_pipeline_smoke.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/launch_contracts.test)
  add_rostest(test/synthetic_pipeline_smoke.test)
endif()
