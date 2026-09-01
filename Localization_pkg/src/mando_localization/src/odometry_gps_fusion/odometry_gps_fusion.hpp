/*
odometry_gps_fusion.hpp
- 역할: NavSatFix 품질을 검사하고 WGS84 위치를 Global EKF용 map frame
        PoseWithCovarianceStamped 측정값으로 변환한다.
- 기준점: first_fix는 첫 유효 GPS를 map 원점으로, manual_datum은 실측 datum을
          map 원점으로 사용한다.
- 단위/frame: 입력 degree·m, 출력 map frame m. yaw_offset_rad는 ENU를 map 축으로 회전한다.
- 실패 경로: datum 미측정, no-fix, stale timestamp, 큰 covariance와 비정상 좌표는 폐기한다.
*/
#pragma once

#include <cstddef>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mando_localization/GpsGateReanchor.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

#include "common/consecutive_recovery_gate.hpp"

namespace mando_localization {

class OdometryGpsFusion {
 public:
  OdometryGpsFusion(const ros::NodeHandle& node,
                    const ros::NodeHandle& private_node);

 private:
  struct Datum {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
    double map_x_m = 0.0;
    double map_y_m = 0.0;
    double map_z_m = 0.0;
    bool ready = false;
  };

  struct ProjectedPoint {
    double x_m = 0.0;
    double y_m = 0.0;
    double z_m = 0.0;
  };

  void load_configuration();
  void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& message);
  void local_odometry_callback(const nav_msgs::Odometry::ConstPtr& message);
  void reanchor_callback(
      const mando_localization::GpsGateReanchor::ConstPtr& message);
  bool validate_fix(const sensor_msgs::NavSatFix& message,
                    std::string* reason) const;
  bool read_covariance(const sensor_msgs::NavSatFix& message,
                       double* variance_east, double* covariance_east_north,
                       double* variance_north, double* variance_up,
                       std::string* reason) const;
  ProjectedPoint project_to_map(const sensor_msgs::NavSatFix& message) const;
  geometry_msgs::PoseWithCovarianceStamped make_pose(
      const sensor_msgs::NavSatFix& message,
      const ProjectedPoint& point) const;
  bool local_odometry_is_fresh(const ros::Time& now,
                               std::string* reason) const;
  bool innovation_is_acceptable(
      const geometry_msgs::PoseWithCovarianceStamped& candidate,
      std::string* reason) const;
  void publish_relocalizing(bool relocalizing);

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher pose_publisher_;
  ros::Publisher candidate_publisher_;
  ros::Publisher relocalizing_publisher_;
  ros::Publisher reanchor_accepted_publisher_;
  ros::Subscriber gps_subscriber_;
  ros::Subscriber local_odometry_subscriber_;
  ros::Subscriber reanchor_subscriber_;

  std::string gps_topic_;
  std::string pose_topic_;
  std::string local_odometry_topic_;
  std::string relocalizing_topic_;
  std::string candidate_topic_;
  std::string reanchor_topic_;
  std::string reanchor_accepted_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_link_frame_;
  std::string gps_frame_;
  std::string reference_mode_;
  Datum datum_;
  bool datum_measured_ = false;
  double yaw_offset_rad_ = 0.0;

  int minimum_fix_status_ = 0;
  double max_message_age_sec_ = 0.0;
  double max_future_stamp_sec_ = 0.0;
  double fallback_horizontal_variance_m2_ = 0.0;
  double fallback_vertical_variance_m2_ = 0.0;
  double max_horizontal_variance_m2_ = 0.0;
  double max_vertical_variance_m2_ = 0.0;
  double max_step_distance_m_ = 0.0;
  double max_position_innovation_m_ = 0.0;
  double max_mahalanobis_distance_ = 0.0;
  double max_reanchor_candidate_distance_m_ = 0.0;
  double local_odometry_timeout_sec_ = 0.0;
  double gps_recovery_gap_sec_ = 0.0;
  double unobserved_variance_ = 0.0;
  ConsecutiveRecoveryGate recovery_gate_{1};

  bool have_last_stamp_ = false;
  ros::Time last_stamp_;
  bool have_last_candidate_point_ = false;
  ProjectedPoint last_candidate_point_;
  bool have_local_odometry_ = false;
  nav_msgs::Odometry local_odometry_;
  ros::Time local_odometry_receipt_time_;
  bool have_prediction_anchor_ = false;
  bool have_latest_quality_candidate_ = false;
  geometry_msgs::PoseWithCovarianceStamped latest_quality_candidate_;
  ProjectedPoint prediction_anchor_map_;
  nav_msgs::Odometry prediction_anchor_local_odometry_;
};

}  // namespace mando_localization
