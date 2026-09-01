/*
odometry_gps_fusion.cpp
- 역할: GPS quality gate와 소규모 주행 영역용 WGS84 local tangent projection을 구현한다.
- 정확도 경계: map datum이 대회 지도 기준점과 일치해야 절대 map 위치가 맞는다.
*/
#include "odometry_gps_fusion/odometry_gps_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <sensor_msgs/NavSatStatus.h>

#include "common/message_validation.hpp"
#include "common/parameter_utils.hpp"

namespace mando_localization {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84EccentricitySquared = 6.69437999014e-3;

double degreesToRadians(double degrees) { return degrees * kPi / 180.0; }

bool validLatitudeLongitude(double latitude_deg, double longitude_deg) {
  return std::isfinite(latitude_deg) && std::isfinite(longitude_deg) &&
         latitude_deg >= -90.0 && latitude_deg <= 90.0 &&
         longitude_deg >= -180.0 && longitude_deg <= 180.0;
}

}  // namespace

OdometryGpsFusion::OdometryGpsFusion(const ros::NodeHandle& node,
                                     const ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {
  load_configuration();
  pose_publisher_ =
      node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          pose_topic_, 10, false);
  candidate_publisher_ =
      node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          candidate_topic_, 10, false);
  relocalizing_publisher_ =
      node_.advertise<std_msgs::Bool>(relocalizing_topic_, 1, true);
  reanchor_accepted_publisher_ =
      node_.advertise<mando_localization::GpsGateReanchor>(
          reanchor_accepted_topic_, 1, false);
  gps_subscriber_ = node_.subscribe(
      gps_topic_, 20, &OdometryGpsFusion::gps_callback, this);
  local_odometry_subscriber_ = node_.subscribe(
      local_odometry_topic_, 50,
      &OdometryGpsFusion::local_odometry_callback, this);
  reanchor_subscriber_ = node_.subscribe(
      reanchor_topic_, 5, &OdometryGpsFusion::reanchor_callback, this);
  publish_relocalizing(false);
  ROS_INFO_STREAM("OdometryGpsFusion 설정: " << gps_topic_ << " -> "
                  << pose_topic_ << ", datum_mode=" << reference_mode_
                  << ", measured=" << (datum_measured_ ? "true" : "false")
                  << ", innovation=" << max_position_innovation_m_ << " m");
}

// 함수이름: load_configuration
// 기능: GPS 토픽, map datum, frame과 품질 기준을 읽어 시작 전에 검증한다.
// 인자: 없음
// 반환값: 없음
void OdometryGpsFusion::load_configuration() {
  gps_topic_ = requireParameter<std::string>(private_node_, "topics/gps_fix");
  pose_topic_ = requireParameter<std::string>(
      private_node_, "internal_topics/gps_gate_pose");
  local_odometry_topic_ = requireParameter<std::string>(
      private_node_, "topics/local_odometry");
  relocalizing_topic_ = requireParameter<std::string>(
      private_node_, "internal_topics/gps_gate_relocalizing");
  candidate_topic_ = requireParameter<std::string>(
      private_node_, "internal_topics/gps_candidate_pose");
  reanchor_topic_ = requireParameter<std::string>(
      private_node_, "internal_topics/gps_reanchor_pose");
  reanchor_accepted_topic_ = requireParameter<std::string>(
      private_node_, "internal_topics/gps_reanchor_accepted");
  map_frame_ = requireParameter<std::string>(private_node_, "frames/map");
  odom_frame_ = requireParameter<std::string>(private_node_, "frames/odom");
  base_link_frame_ =
      requireParameter<std::string>(private_node_, "frames/base_link");
  gps_frame_ = requireParameter<std::string>(private_node_, "frames/gps");
  reference_mode_ =
      requireParameter<std::string>(private_node_, "reference/mode");
  datum_measured_ =
      requireParameter<bool>(private_node_, "reference/measured");
  yaw_offset_rad_ =
      requireParameter<double>(private_node_, "reference/yaw_offset_rad");

  minimum_fix_status_ = requireParameter<int>(
      private_node_, "quality/minimum_fix_status");
  max_message_age_sec_ = requireParameter<double>(
      private_node_, "quality/max_message_age_sec");
  max_future_stamp_sec_ = requireParameter<double>(
      private_node_, "quality/max_future_stamp_sec");
  fallback_horizontal_variance_m2_ = requireParameter<double>(
      private_node_, "quality/fallback_horizontal_variance_m2");
  fallback_vertical_variance_m2_ = requireParameter<double>(
      private_node_, "quality/fallback_vertical_variance_m2");
  max_horizontal_variance_m2_ = requireParameter<double>(
      private_node_, "quality/max_horizontal_variance_m2");
  max_vertical_variance_m2_ = requireParameter<double>(
      private_node_, "quality/max_vertical_variance_m2");
  max_step_distance_m_ = requireParameter<double>(
      private_node_, "quality/max_step_distance_m");
  max_position_innovation_m_ = requireParameter<double>(
      private_node_, "quality/max_position_innovation_m");
  max_mahalanobis_distance_ = requireParameter<double>(
      private_node_, "quality/max_mahalanobis_distance");
  max_reanchor_candidate_distance_m_ = requireParameter<double>(
      private_node_, "quality/max_reanchor_candidate_distance_m");
  local_odometry_timeout_sec_ = requireParameter<double>(
      private_node_, "sources/local_odometry/timeout_sec");
  gps_recovery_gap_sec_ = requireParameter<double>(
      private_node_, "sources/gps/timeout_sec");
  unobserved_variance_ = requireParameter<double>(
      private_node_, "quality/unobserved_variance");
  const int required_consecutive_fixes = requireParameter<int>(
      private_node_, "absolute_sources/recovery_consecutive_measurements");

  requireAbsoluteRosName(gps_topic_, "topics/gps_fix");
  requireAbsoluteRosName(pose_topic_, "internal_topics/gps_gate_pose");
  requireAbsoluteRosName(local_odometry_topic_, "topics/local_odometry");
  requireAbsoluteRosName(relocalizing_topic_,
                         "internal_topics/gps_gate_relocalizing");
  requireAbsoluteRosName(candidate_topic_,
                         "internal_topics/gps_candidate_pose");
  requireAbsoluteRosName(reanchor_topic_,
                         "internal_topics/gps_reanchor_pose");
  requireAbsoluteRosName(reanchor_accepted_topic_,
                         "internal_topics/gps_reanchor_accepted");
  if (gps_topic_ == pose_topic_) {
    throw std::runtime_error("GPS input and map pose output topics must differ");
  }
  requireNonEmpty(map_frame_, "frames/map");
  requireNonEmpty(odom_frame_, "frames/odom");
  requireNonEmpty(base_link_frame_, "frames/base_link");
  requireNonEmpty(gps_frame_, "frames/gps");
  if (reference_mode_ != "first_fix" && reference_mode_ != "manual_datum") {
    throw std::runtime_error(
        "reference/mode must be first_fix or manual_datum");
  }
  if (!std::isfinite(yaw_offset_rad_)) {
    throw std::runtime_error("reference/yaw_offset_rad must be finite");
  }
  if (minimum_fix_status_ < sensor_msgs::NavSatStatus::STATUS_FIX ||
      minimum_fix_status_ > sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) {
    throw std::runtime_error("quality/minimum_fix_status is out of range");
  }
  requireFinitePositive(max_message_age_sec_,
                        "quality/max_message_age_sec");
  requireFiniteNonnegative(max_future_stamp_sec_,
                           "quality/max_future_stamp_sec");
  requireFinitePositive(fallback_horizontal_variance_m2_,
                        "quality/fallback_horizontal_variance_m2");
  requireFinitePositive(fallback_vertical_variance_m2_,
                        "quality/fallback_vertical_variance_m2");
  requireFinitePositive(max_horizontal_variance_m2_,
                        "quality/max_horizontal_variance_m2");
  requireFinitePositive(max_vertical_variance_m2_,
                        "quality/max_vertical_variance_m2");
  if (fallback_horizontal_variance_m2_ > max_horizontal_variance_m2_) {
    throw std::runtime_error(
        "quality/fallback_horizontal_variance_m2 must not exceed "
        "quality/max_horizontal_variance_m2");
  }
  if (fallback_vertical_variance_m2_ > max_vertical_variance_m2_) {
    throw std::runtime_error(
        "quality/fallback_vertical_variance_m2 must not exceed "
        "quality/max_vertical_variance_m2");
  }
  requireFinitePositive(max_step_distance_m_,
                        "quality/max_step_distance_m");
  requireFinitePositive(max_position_innovation_m_,
                        "quality/max_position_innovation_m");
  requireFinitePositive(max_mahalanobis_distance_,
                        "quality/max_mahalanobis_distance");
  requireFinitePositive(max_reanchor_candidate_distance_m_,
                        "quality/max_reanchor_candidate_distance_m");
  requireFinitePositive(local_odometry_timeout_sec_,
                        "sources/local_odometry/timeout_sec");
  requireFinitePositive(gps_recovery_gap_sec_,
                        "sources/gps/timeout_sec");
  requireFinitePositive(unobserved_variance_,
                        "quality/unobserved_variance");
  if (required_consecutive_fixes <= 0) {
    throw std::runtime_error(
        "absolute_sources/recovery_consecutive_measurements must be positive");
  }
  recovery_gate_ = ConsecutiveRecoveryGate(required_consecutive_fixes);

  if (reference_mode_ == "manual_datum") {
    if (!datum_measured_) {
      throw std::runtime_error(
          "manual_datum requires reference/measured=true");
    }
    datum_.latitude_deg =
        requireParameter<double>(private_node_, "reference/latitude_deg");
    datum_.longitude_deg =
        requireParameter<double>(private_node_, "reference/longitude_deg");
    datum_.altitude_m =
        requireParameter<double>(private_node_, "reference/altitude_m");
    datum_.map_x_m =
        requireParameter<double>(private_node_, "reference/map_x_m");
    datum_.map_y_m =
        requireParameter<double>(private_node_, "reference/map_y_m");
    datum_.map_z_m =
        requireParameter<double>(private_node_, "reference/map_z_m");
    const std::string measured_at = requireParameter<std::string>(
        private_node_, "reference/measured_at");
    const std::string source = requireParameter<std::string>(
        private_node_, "reference/source");
    if (!validLatitudeLongitude(datum_.latitude_deg, datum_.longitude_deg) ||
        !std::isfinite(datum_.altitude_m) ||
        !std::isfinite(datum_.map_x_m) || !std::isfinite(datum_.map_y_m) ||
        !std::isfinite(datum_.map_z_m) || measured_at.empty() ||
        source.empty()) {
      throw std::runtime_error("manual datum values must be finite WGS84 data");
    }
    datum_.ready = true;
  }
}

void OdometryGpsFusion::local_odometry_callback(
    const nav_msgs::Odometry::ConstPtr& message) {
  const bool valid = !message->header.stamp.isZero() &&
                     message->header.frame_id == odom_frame_ &&
                     message->child_frame_id == base_link_frame_ &&
                     std::isfinite(message->pose.pose.position.x) &&
                     std::isfinite(message->pose.pose.position.y);
  if (!valid) {
    ROS_WARN_THROTTLE(1.0, "Rejecting local odometry used by GPS gate");
    return;
  }
  std::string covariance_reason;
  if (!MessageValidation::validateCovariance(
          message->pose.covariance.data(), 6, false,
          std::numeric_limits<double>::max(), &covariance_reason)) {
    ROS_WARN_THROTTLE(1.0, "Rejecting invalid local odometry covariance: %s",
                      covariance_reason.c_str());
    return;
  }
  local_odometry_ = *message;
  local_odometry_receipt_time_ = ros::Time::now();
  have_local_odometry_ = true;
}

void OdometryGpsFusion::reanchor_callback(
    const mando_localization::GpsGateReanchor::ConstPtr& message) {
  const geometry_msgs::PoseWithCovarianceStamped& pose = message->pose;
  std::string reason;
  std::string covariance_reason;
  const bool covariance_valid = MessageValidation::validateCovariance(
      pose.pose.covariance.data(), 6, false,
      std::numeric_limits<double>::max(), &covariance_reason);
  const double candidate_distance = have_latest_quality_candidate_
      ? std::hypot(
            pose.pose.pose.position.x -
                latest_quality_candidate_.pose.pose.position.x,
            pose.pose.pose.position.y -
                latest_quality_candidate_.pose.pose.position.y)
      : std::numeric_limits<double>::infinity();
  const double candidate_stamp_gap = have_latest_quality_candidate_
      ? std::abs((pose.header.stamp -
                  latest_quality_candidate_.header.stamp).toSec())
      : std::numeric_limits<double>::infinity();
  if (message->transaction_id == 0U || pose.header.stamp.isZero() ||
      pose.header.frame_id != map_frame_ ||
      !std::isfinite(pose.pose.pose.position.x) ||
      !std::isfinite(pose.pose.pose.position.y) ||
      !std::isfinite(pose.pose.pose.position.z) ||
      !MessageValidation::finiteQuaternion(pose.pose.pose.orientation) ||
      !covariance_valid ||
      !have_latest_quality_candidate_ ||
      candidate_distance > max_reanchor_candidate_distance_m_ ||
      candidate_stamp_gap > gps_recovery_gap_sec_ ||
      !local_odometry_is_fresh(ros::Time::now(), &reason)) {
    if (reason.empty() && !covariance_valid) {
      reason = covariance_reason;
    }
    ROS_WARN_THROTTLE(1.0, "Rejecting GPS gate reanchor: %s",
                      reason.empty() ? "invalid_reanchor_pose" : reason.c_str());
    return;
  }

  prediction_anchor_map_.x_m = pose.pose.pose.position.x;
  prediction_anchor_map_.y_m = pose.pose.pose.position.y;
  prediction_anchor_map_.z_m = pose.pose.pose.position.z;
  prediction_anchor_local_odometry_ = local_odometry_;
  have_prediction_anchor_ = true;
  last_candidate_point_ = prediction_anchor_map_;
  have_last_candidate_point_ = true;
  last_stamp_ = pose.header.stamp;
  have_last_stamp_ = true;
  recovery_gate_.forceAccept();
  publish_relocalizing(false);
  // Coordinator는 이 적용 확인을 받은 뒤에만 GPS pose를 공개한다.
  reanchor_accepted_publisher_.publish(*message);
  ROS_INFO("GPS gate prediction anchor was restored by RelocalizationCoordinator");
}

bool OdometryGpsFusion::local_odometry_is_fresh(
    const ros::Time& now, std::string* reason) const {
  if (!have_local_odometry_ || local_odometry_receipt_time_.isZero()) {
    *reason = "local_odometry_not_received";
    return false;
  }
  const double receipt_age = (now - local_odometry_receipt_time_).toSec();
  const double stamp_age = (now - local_odometry_.header.stamp).toSec();
  if (!std::isfinite(receipt_age) || !std::isfinite(stamp_age) ||
      receipt_age < 0.0 || receipt_age > local_odometry_timeout_sec_ ||
      stamp_age < -max_future_stamp_sec_ ||
      stamp_age > local_odometry_timeout_sec_) {
    *reason = "local_odometry_stale";
    return false;
  }
  reason->clear();
  return true;
}

bool OdometryGpsFusion::innovation_is_acceptable(
    const geometry_msgs::PoseWithCovarianceStamped& candidate,
    std::string* reason) const {
  if (!have_prediction_anchor_) {
    reason->clear();
    return true;
  }

  const double local_dx = local_odometry_.pose.pose.position.x -
                          prediction_anchor_local_odometry_.pose.pose.position.x;
  const double local_dy = local_odometry_.pose.pose.position.y -
                          prediction_anchor_local_odometry_.pose.pose.position.y;
  const double cosine = std::cos(yaw_offset_rad_);
  const double sine = std::sin(yaw_offset_rad_);
  const double predicted_x = prediction_anchor_map_.x_m +
                             cosine * local_dx - sine * local_dy;
  const double predicted_y = prediction_anchor_map_.y_m +
                             sine * local_dx + cosine * local_dy;
  const double dx = candidate.pose.pose.position.x - predicted_x;
  const double dy = candidate.pose.pose.position.y - predicted_y;
  if (std::hypot(dx, dy) > max_position_innovation_m_) {
    *reason = "gps_position_innovation_too_large";
    return false;
  }

  const double local_xx = local_odometry_.pose.covariance[0];
  const double local_xy = 0.5 * (local_odometry_.pose.covariance[1] +
                                 local_odometry_.pose.covariance[6]);
  const double local_yy = local_odometry_.pose.covariance[7];
  const double rotated_xx = cosine * cosine * local_xx +
                            sine * sine * local_yy -
                            2.0 * sine * cosine * local_xy;
  const double rotated_yy = sine * sine * local_xx +
                            cosine * cosine * local_yy +
                            2.0 * sine * cosine * local_xy;
  const double rotated_xy = sine * cosine * (local_xx - local_yy) +
                            (cosine * cosine - sine * sine) * local_xy;
  const double covariance_xx = candidate.pose.covariance[0] + rotated_xx;
  const double covariance_xy = candidate.pose.covariance[1] + rotated_xy;
  const double covariance_yy = candidate.pose.covariance[7] + rotated_yy;
  const double determinant = covariance_xx * covariance_yy -
                             covariance_xy * covariance_xy;
  if (!std::isfinite(determinant) || determinant <= 1.0e-12) {
    *reason = "gps_innovation_covariance_invalid";
    return false;
  }
  const double mahalanobis_squared =
      (covariance_yy * dx * dx - 2.0 * covariance_xy * dx * dy +
       covariance_xx * dy * dy) /
      determinant;
  if (!std::isfinite(mahalanobis_squared) || mahalanobis_squared < 0.0 ||
      std::sqrt(mahalanobis_squared) > max_mahalanobis_distance_) {
    *reason = "gps_mahalanobis_innovation_too_large";
    return false;
  }
  reason->clear();
  return true;
}

void OdometryGpsFusion::publish_relocalizing(const bool relocalizing) {
  std_msgs::Bool message;
  message.data = relocalizing;
  relocalizing_publisher_.publish(message);
}

// 함수이름: validate_fix
// 기능: GPS fix 상태, frame, timestamp, WGS84 범위와 covariance를 검사한다.
// 인자: message, reason
// 반환값: map pose 후보로 사용할 수 있으면 true
bool OdometryGpsFusion::validate_fix(const sensor_msgs::NavSatFix& message,
                                     std::string* reason) const {
  if (message.status.status < minimum_fix_status_) {
    *reason = "gps_no_fix";
    return false;
  }
  if (message.header.frame_id != gps_frame_) {
    *reason = "gps_frame_mismatch";
    return false;
  }
  if (!MessageValidation::validateStamp(
          message.header.stamp, ros::Time::now(), max_message_age_sec_,
          max_future_stamp_sec_, have_last_stamp_, last_stamp_, reason)) {
    return false;
  }
  if (!validLatitudeLongitude(message.latitude, message.longitude) ||
      !std::isfinite(message.altitude)) {
    *reason = "gps_coordinates_invalid";
    return false;
  }
  double variance_east = 0.0;
  double covariance_east_north = 0.0;
  double variance_north = 0.0;
  double variance_up = 0.0;
  return read_covariance(message, &variance_east, &covariance_east_north,
                         &variance_north, &variance_up, reason);
}

// 함수이름: read_covariance
// 기능: NavSatFix ENU covariance를 읽고 unknown인 경우 설정 fallback을 사용한다.
// 인자: message와 ENU covariance 출력 포인터
// 반환값: 사용할 covariance가 품질 기준을 만족하면 true
bool OdometryGpsFusion::read_covariance(
    const sensor_msgs::NavSatFix& message, double* variance_east,
    double* covariance_east_north, double* variance_north,
    double* variance_up, std::string* reason) const {
  if (variance_east == nullptr || covariance_east_north == nullptr ||
      variance_north == nullptr || variance_up == nullptr) {
    *reason = "gps_covariance_output_invalid";
    return false;
  }
  if (message.position_covariance_type ==
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    *variance_east = fallback_horizontal_variance_m2_;
    *covariance_east_north = 0.0;
    *variance_north = fallback_horizontal_variance_m2_;
    *variance_up = fallback_vertical_variance_m2_;
    reason->clear();
    return true;
  }
  if (message.position_covariance_type <
          sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED ||
      message.position_covariance_type >
          sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN) {
    *reason = "gps_covariance_type_invalid";
    return false;
  }
  if (!MessageValidation::validateCovariance(
          message.position_covariance.data(), 3, false,
          std::max(max_horizontal_variance_m2_, max_vertical_variance_m2_),
          reason)) {
    *reason = "gps_" + *reason;
    return false;
  }
  *variance_east = message.position_covariance[0];
  *covariance_east_north =
      0.5 * (message.position_covariance[1] +
             message.position_covariance[3]);
  *variance_north = message.position_covariance[4];
  *variance_up = message.position_covariance[8];
  if (*variance_east > max_horizontal_variance_m2_ ||
      *variance_north > max_horizontal_variance_m2_) {
    *reason = "gps_horizontal_variance_too_large";
    return false;
  }
  if (*variance_up > max_vertical_variance_m2_) {
    *reason = "gps_vertical_variance_too_large";
    return false;
  }
  reason->clear();
  return true;
}

// 함수이름: project_to_map
// 기능: datum 주변 WGS84 차이를 타원체 곡률로 ENU에 투영한 뒤 map yaw를 적용한다.
// 인자: message
// 반환값: map frame 위치 m
OdometryGpsFusion::ProjectedPoint OdometryGpsFusion::project_to_map(
    const sensor_msgs::NavSatFix& message) const {
  const double latitude_rad = degreesToRadians(datum_.latitude_deg);
  const double sin_latitude = std::sin(latitude_rad);
  const double denominator =
      std::sqrt(1.0 - kWgs84EccentricitySquared *
                          sin_latitude * sin_latitude);
  const double prime_vertical_radius_m =
      kWgs84SemiMajorAxisM / denominator;
  const double meridian_radius_m =
      kWgs84SemiMajorAxisM * (1.0 - kWgs84EccentricitySquared) /
      (denominator * denominator * denominator);
  const double east_m =
      prime_vertical_radius_m * std::cos(latitude_rad) *
      degreesToRadians(message.longitude - datum_.longitude_deg);
  const double north_m =
      meridian_radius_m *
      degreesToRadians(message.latitude - datum_.latitude_deg);
  const double cosine = std::cos(yaw_offset_rad_);
  const double sine = std::sin(yaw_offset_rad_);

  ProjectedPoint point;
  point.x_m = datum_.map_x_m + cosine * east_m - sine * north_m;
  point.y_m = datum_.map_y_m + sine * east_m + cosine * north_m;
  point.z_m = datum_.map_z_m + message.altitude - datum_.altitude_m;
  return point;
}

// 함수이름: make_pose
// 기능: 투영 위치와 회전된 GPS ENU covariance를 map pose 측정값으로 만든다.
// 인자: message, point
// 반환값: Global EKF 입력용 PoseWithCovarianceStamped
geometry_msgs::PoseWithCovarianceStamped OdometryGpsFusion::make_pose(
    const sensor_msgs::NavSatFix& message,
    const ProjectedPoint& point) const {
  double variance_east = 0.0;
  double covariance_east_north = 0.0;
  double variance_north = 0.0;
  double variance_up = 0.0;
  std::string ignored_reason;
  read_covariance(message, &variance_east, &covariance_east_north,
                  &variance_north, &variance_up, &ignored_reason);
  const double cosine = std::cos(yaw_offset_rad_);
  const double sine = std::sin(yaw_offset_rad_);

  geometry_msgs::PoseWithCovarianceStamped output;
  output.header.stamp = message.header.stamp;
  output.header.frame_id = map_frame_;
  output.pose.pose.position.x = point.x_m;
  output.pose.pose.position.y = point.y_m;
  output.pose.pose.position.z = point.z_m;
  output.pose.pose.orientation.w = 1.0;
  std::fill(output.pose.covariance.begin(), output.pose.covariance.end(), 0.0);
  output.pose.covariance[0] =
      cosine * cosine * variance_east + sine * sine * variance_north -
      2.0 * sine * cosine * covariance_east_north;
  output.pose.covariance[7] =
      sine * sine * variance_east + cosine * cosine * variance_north +
      2.0 * sine * cosine * covariance_east_north;
  const double xy_covariance =
      sine * cosine * (variance_east - variance_north) +
      (cosine * cosine - sine * sine) * covariance_east_north;
  output.pose.covariance[1] = xy_covariance;
  output.pose.covariance[6] = xy_covariance;
  output.pose.covariance[14] = variance_up;
  output.pose.covariance[21] = unobserved_variance_;
  output.pose.covariance[28] = unobserved_variance_;
  output.pose.covariance[35] = unobserved_variance_;
  return output;
}

// 함수이름: gps_callback
// 기능: 유효 GPS를 datum에 투영하고 연속 fix gate를 통과한 pose만 발행한다.
// 인자: message
// 반환값: 없음
void OdometryGpsFusion::gps_callback(
    const sensor_msgs::NavSatFix::ConstPtr& message) {
  std::string reason;
  if (!validate_fix(*message, &reason)) {
    recovery_gate_.markUnhealthy();
    have_last_candidate_point_ = false;
    publish_relocalizing(recovery_gate_.recovering());
    ROS_WARN_THROTTLE(1.0, "Rejecting GPS fix: %s", reason.c_str());
    return;
  }
  if (!local_odometry_is_fresh(ros::Time::now(), &reason)) {
    recovery_gate_.markUnhealthy();
    have_last_candidate_point_ = false;
    publish_relocalizing(recovery_gate_.recovering());
    ROS_WARN_THROTTLE(1.0, "Rejecting GPS fix: %s", reason.c_str());
    return;
  }

  if (have_last_stamp_ && recovery_gate_.acceptedOnce()) {
    const double stamp_gap_sec =
        (message->header.stamp - last_stamp_).toSec();
    // rosbag 반복 재생 등 시간 epoch가 바뀐 경우도 단절과 동일하게 재검증한다.
    if (stamp_gap_sec < 0.0 || stamp_gap_sec > gps_recovery_gap_sec_) {
      recovery_gate_.markUnhealthy();
      have_last_candidate_point_ = false;
    }
  }

  if (!datum_.ready) {
    datum_.latitude_deg = message->latitude;
    datum_.longitude_deg = message->longitude;
    datum_.altitude_m = message->altitude;
    datum_.map_x_m = 0.0;
    datum_.map_y_m = 0.0;
    datum_.map_z_m = 0.0;
    datum_.ready = true;
    ROS_INFO("First valid GPS fix established the local map datum");
  }

  const ProjectedPoint point = project_to_map(*message);
  if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) ||
      !std::isfinite(point.z_m)) {
    recovery_gate_.markUnhealthy();
    have_last_candidate_point_ = false;
    publish_relocalizing(recovery_gate_.recovering());
    ROS_WARN_THROTTLE(1.0, "Rejecting non-finite projected GPS pose");
    return;
  }
  const geometry_msgs::PoseWithCovarianceStamped candidate =
      make_pose(*message, point);
  latest_quality_candidate_ = candidate;
  have_latest_quality_candidate_ = true;
  // 승인 전 후보는 Coordinator 교차검증에만 사용한다. InterfaceAdapter가
  // Global EKF로 relay하는 공개 GPS pose와는 의도적으로 분리한다.
  candidate_publisher_.publish(candidate);
  if (!innovation_is_acceptable(candidate, &reason)) {
    recovery_gate_.markUnhealthy();
    have_last_candidate_point_ = false;
    publish_relocalizing(recovery_gate_.recovering());
    ROS_WARN_THROTTLE(1.0, "Rejecting GPS fix: %s", reason.c_str());
    return;
  }
  if (have_last_candidate_point_ &&
      std::hypot(point.x_m - last_candidate_point_.x_m,
                 point.y_m - last_candidate_point_.y_m) >
          max_step_distance_m_) {
    recovery_gate_.resetCandidate();
  }
  last_candidate_point_ = point;
  have_last_candidate_point_ = true;

  last_stamp_ = message->header.stamp;
  have_last_stamp_ = true;
  if (!recovery_gate_.observeHealthy()) {
    publish_relocalizing(recovery_gate_.recovering());
    return;
  }
  pose_publisher_.publish(candidate);
  prediction_anchor_map_ = point;
  prediction_anchor_local_odometry_ = local_odometry_;
  have_prediction_anchor_ = true;
  publish_relocalizing(false);
}

}  // namespace mando_localization
