#include <ros/ros.h>

#include <exception>

#include "localization_status_manager.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_status_manager");
  try {
    mando_localization::LocalizationStatusManager manager(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("LocalizationStatusManager 시작 실패: " << error.what());
    return 1;
  }
  return 0;
}
