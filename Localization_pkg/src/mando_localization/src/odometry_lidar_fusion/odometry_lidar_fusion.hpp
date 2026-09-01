/*
odometry_lidar_fusion.hpp
- 역할: LaserScan을 검증해 AMCL 입력으로 relay하고, AMCL이 odometry·지도·LiDAR로
        계산한 pose를 품질 gate하여 Global EKF용 map pose로 발행한다.
- 단위/frame: scan은 laser_link의 m/rad, pose는 map frame의 m/rad다.
- 초기화: 수동 /initialpose는 AMCL이 직접 처리하며, 선택적으로 검증된 GPS map pose를
          설정된 초기 yaw와 함께 /initialpose로 한 번 전달한다.
- 실패 경로: scan/pose frame 불일치, stale timestamp, NaN, 큰 covariance와 불안정한
             위치 점프는 출력하지 않는다.
*/
#pragma once

#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include "common/consecutive_recovery_gate.hpp"

namespace mando_localization {

class OdometryLidarFusion {
 public:
  OdometryLidarFusion(const ros::NodeHandle& node,
                      const ros::NodeHandle& private_node);

 private:
  void load_configuration();
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& message);
  void amcl_pose_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message);
  void gps_pose_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message);

  bool validate_scan(const sensor_msgs::LaserScan& message,
                     std::string* reason) const;
  bool validate_map_pose(const geometry_msgs::PoseWithCovarianceStamped& message,
                         bool check_monotonic_stamp,
                         bool require_yaw_quality,
                         std::string* reason) const;
  bool pose_is_stable_candidate(
      const geometry_msgs::PoseWithCovarianceStamped& message) const;
  bool accept_pose_candidate(
      const geometry_msgs::PoseWithCovarianceStamped& message);
  void publish_relocalizing(bool relocalizing);

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher scan_publisher_;
  ros::Publisher lidar_pose_publisher_;
  ros::Publisher initialpose_publisher_;
  ros::Publisher relocalizing_publisher_;
  ros::Subscriber scan_subscriber_;
  ros::Subscriber amcl_pose_subscriber_;
  ros::Subscriber gps_pose_subscriber_;

  std::string scan_input_topic_;
  std::string scan_output_topic_;
  std::string amcl_pose_topic_;
  std::string lidar_pose_topic_;
  std::string gps_pose_topic_;
  std::string initialpose_topic_;
  std::string relocalizing_topic_;
  std::string map_frame_;
  std::string lidar_frame_;

  double scan_max_message_age_sec_ = 0.0;
  int min_range_count_ = 0;
  double min_finite_ratio_ = 0.0;
  bool require_laser_frame_ = true;
  double pose_max_message_age_sec_ = 0.0;
  double max_future_stamp_sec_ = 0.0;
  double max_position_variance_m2_ = 0.0;
  double max_yaw_variance_rad2_ = 0.0;
  double max_pose_jump_m_ = 0.0;
  double max_yaw_jump_rad_ = 0.0;
  double stable_candidate_radius_m_ = 0.0;
  double stable_candidate_yaw_rad_ = 0.0;
  double min_quaternion_norm_ = 0.0;
  double max_quaternion_normalization_error_ = 0.0;
  double recovery_gap_sec_ = 0.0;
  ConsecutiveRecoveryGate recovery_gate_{1};

  bool use_gps_initialpose_ = false;
  double gps_initial_yaw_rad_ = 0.0;
  double gps_initial_yaw_variance_rad2_ = 0.0;
  std::string gps_initial_yaw_calibration_state_;
  bool gps_initialpose_published_ = false;

  bool have_last_scan_stamp_ = false;
  ros::Time last_scan_stamp_;
  bool have_last_pose_stamp_ = false;
  ros::Time last_pose_stamp_;
  bool have_accepted_pose_ = false;
  geometry_msgs::PoseWithCovarianceStamped accepted_pose_;
  bool have_candidate_pose_ = false;
  geometry_msgs::PoseWithCovarianceStamped candidate_pose_;
};

}  // namespace mando_localization
