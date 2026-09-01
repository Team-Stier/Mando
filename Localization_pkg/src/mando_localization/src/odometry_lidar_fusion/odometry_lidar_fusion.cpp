/*
odometry_lidar_fusion.cpp
- 역할: LaserScan relay와 AMCL pose의 covariance·연속성 검사를 구현한다.
- 책임 경계: scan matching과 particle filter 계산은 AMCL, 최종 다중 센서 융합은
             robot_localization Global EKF가 담당한다.
*/
#include "odometry_lidar_fusion/odometry_lidar_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "common/message_validation.hpp"
#include "common/parameter_utils.hpp"

namespace mando_localization {
namespace {

constexpr char kInternalScanTopic[] =
    "/mando_localization/internal/amcl/scan";
constexpr char kInternalAmclPoseTopic[] =
    "/mando_localization/internal/amcl/pose";
constexpr char kInternalInitialposeTopic[] =
    "/mando_localization/internal/amcl/initialpose";

}  // namespace

OdometryLidarFusion::OdometryLidarFusion(
    const ros::NodeHandle& node, const ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {
  load_configuration();
  scan_publisher_ =
      node_.advertise<sensor_msgs::LaserScan>(scan_output_topic_, 10, false);
  lidar_pose_publisher_ =
      node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          lidar_pose_topic_, 10, false);
  initialpose_publisher_ =
      node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          initialpose_topic_, 1, true);
  relocalizing_publisher_ =
      node_.advertise<std_msgs::Bool>(relocalizing_topic_, 1, true);
  scan_subscriber_ = node_.subscribe(
      scan_input_topic_, 20, &OdometryLidarFusion::scan_callback, this);
  amcl_pose_subscriber_ = node_.subscribe(
      amcl_pose_topic_, 20, &OdometryLidarFusion::amcl_pose_callback, this);
  if (use_gps_initialpose_) {
    gps_pose_subscriber_ = node_.subscribe(
        gps_pose_topic_, 10, &OdometryLidarFusion::gps_pose_callback, this);
  }
  publish_relocalizing(false);
  ROS_INFO_STREAM("OdometryLidarFusion 설정: scan=" << scan_input_topic_
                  << " -> " << scan_output_topic_ << ", AMCL="
                  << amcl_pose_topic_ << " -> " << lidar_pose_topic_
                  << ", GPS initialpose="
                  << (use_gps_initialpose_ ? "true" : "false"));
}

// 함수이름: load_configuration
// 기능: LiDAR scan relay, AMCL pose gate와 선택적 GPS 초기화 설정을 검증한다.
// 인자: 없음
// 반환값: 없음
void OdometryLidarFusion::load_configuration() {
  scan_input_topic_ = requireParameter<std::string>(
      private_node_, "topics/lidar_scan");
  scan_output_topic_ = kInternalScanTopic;
  amcl_pose_topic_ = kInternalAmclPoseTopic;
  lidar_pose_topic_ =
      requireParameter<std::string>(private_node_, "topics/lidar_map_pose");
  initialpose_topic_ = kInternalInitialposeTopic;
  gps_pose_topic_ =
      requireParameter<std::string>(private_node_, "topics/gps_map_pose");
  relocalizing_topic_ = requireParameter<std::string>(
      private_node_, "topics/lidar_relocalizing");
  map_frame_ = requireParameter<std::string>(private_node_, "frames/map");
  lidar_frame_ = requireParameter<std::string>(private_node_, "frames/lidar");

  scan_max_message_age_sec_ = requireParameter<double>(
      private_node_, "scan_validation/timeout_sec");
  min_range_count_ = requireParameter<int>(
      private_node_, "scan_validation/min_range_count");
  min_finite_ratio_ = requireParameter<double>(
      private_node_, "scan_validation/min_finite_ratio");
  require_laser_frame_ = requireParameter<bool>(
      private_node_, "scan_validation/require_laser_frame");
  pose_max_message_age_sec_ = requireParameter<double>(
      private_node_, "quality/pose_max_message_age_sec");
  max_future_stamp_sec_ = requireParameter<double>(
      private_node_, "quality/max_future_stamp_sec");
  max_position_variance_m2_ = requireParameter<double>(
      private_node_, "quality/max_position_variance_m2");
  max_yaw_variance_rad2_ = requireParameter<double>(
      private_node_, "quality/max_yaw_variance_rad2");
  max_pose_jump_m_ = requireParameter<double>(
      private_node_, "quality/max_pose_jump_m");
  max_yaw_jump_rad_ = requireParameter<double>(
      private_node_, "quality/max_yaw_jump_rad");
  stable_candidate_radius_m_ = requireParameter<double>(
      private_node_, "quality/stable_candidate_radius_m");
  stable_candidate_yaw_rad_ = requireParameter<double>(
      private_node_, "quality/stable_candidate_yaw_rad");
  min_quaternion_norm_ = requireParameter<double>(
      private_node_, "quality/min_quaternion_norm");
  max_quaternion_normalization_error_ = requireParameter<double>(
      private_node_, "quality/max_quaternion_normalization_error");
  const int required_consecutive_poses = requireParameter<int>(
      private_node_, "absolute_sources/recovery_consecutive_measurements");
  recovery_gap_sec_ = requireParameter<double>(
      private_node_, "sources/lidar/timeout_sec");
  use_gps_initialpose_ = requireParameter<bool>(
      private_node_, "initialization/use_gps_initialpose");

  requireAbsoluteRosName(scan_input_topic_, "topics/lidar_scan");
  requireAbsoluteRosName(scan_output_topic_,
                         "internal AMCL scan topic");
  requireAbsoluteRosName(amcl_pose_topic_, "internal AMCL pose topic");
  requireAbsoluteRosName(lidar_pose_topic_, "topics/lidar_map_pose");
  requireAbsoluteRosName(initialpose_topic_, "internal AMCL initialpose topic");
  requireAbsoluteRosName(gps_pose_topic_, "topics/gps_map_pose");
  requireAbsoluteRosName(relocalizing_topic_, "topics/lidar_relocalizing");
  if (scan_input_topic_ == scan_output_topic_) {
    throw std::runtime_error("LiDAR scan input and relay topics must differ");
  }
  if (amcl_pose_topic_ == lidar_pose_topic_) {
    throw std::runtime_error("AMCL input and LiDAR pose output topics must differ");
  }
  requireNonEmpty(map_frame_, "frames/map");
  requireNonEmpty(lidar_frame_, "frames/lidar");
  requireFinitePositive(scan_max_message_age_sec_,
                        "scan_validation/timeout_sec");
  if (min_range_count_ <= 0) {
    throw std::runtime_error(
        "scan_validation/min_range_count must be positive");
  }
  if (!std::isfinite(min_finite_ratio_) || min_finite_ratio_ < 0.0 ||
      min_finite_ratio_ > 1.0) {
    throw std::runtime_error(
        "scan_validation/min_finite_ratio must be within [0, 1]");
  }
  requireFinitePositive(pose_max_message_age_sec_,
                        "quality/pose_max_message_age_sec");
  requireFiniteNonnegative(max_future_stamp_sec_,
                           "quality/max_future_stamp_sec");
  requireFinitePositive(max_position_variance_m2_,
                        "quality/max_position_variance_m2");
  requireFinitePositive(max_yaw_variance_rad2_,
                        "quality/max_yaw_variance_rad2");
  requireFinitePositive(max_pose_jump_m_, "quality/max_pose_jump_m");
  requireFinitePositive(max_yaw_jump_rad_, "quality/max_yaw_jump_rad");
  requireFinitePositive(stable_candidate_radius_m_,
                        "quality/stable_candidate_radius_m");
  requireFinitePositive(stable_candidate_yaw_rad_,
                        "quality/stable_candidate_yaw_rad");
  requireFinitePositive(min_quaternion_norm_,
                        "quality/min_quaternion_norm");
  requireFinitePositive(max_quaternion_normalization_error_,
                        "quality/max_quaternion_normalization_error");
  if (required_consecutive_poses <= 0) {
    throw std::runtime_error(
        "absolute_sources/recovery_consecutive_measurements must be positive");
  }
  requireFinitePositive(recovery_gap_sec_, "sources/lidar/timeout_sec");
  recovery_gate_ = ConsecutiveRecoveryGate(required_consecutive_poses);
  if (use_gps_initialpose_) {
    gps_initial_yaw_rad_ = requireParameter<double>(
        private_node_, "initialization/gps_initial_yaw_rad");
    gps_initial_yaw_variance_rad2_ = requireParameter<double>(
        private_node_, "initialization/gps_initial_yaw_variance_rad2");
    gps_initial_yaw_calibration_state_ = requireParameter<std::string>(
        private_node_, "initialization/gps_initial_yaw_calibration_state");
    const bool require_measured_datum = requireParameter<bool>(
        private_node_, "reference/require_measured_datum_for_lidar");
    const bool datum_measured =
        requireParameter<bool>(private_node_, "reference/measured");
    if (!std::isfinite(gps_initial_yaw_rad_)) {
      throw std::runtime_error(
          "initialization/gps_initial_yaw_rad must be finite");
    }
    requireFinitePositive(
        gps_initial_yaw_variance_rad2_,
        "initialization/gps_initial_yaw_variance_rad2");
    if (gps_initial_yaw_calibration_state_ != "unmeasured" &&
        gps_initial_yaw_calibration_state_ != "measured" &&
        gps_initial_yaw_calibration_state_ != "verified") {
      throw std::runtime_error(
          "initialization/gps_initial_yaw_calibration_state is invalid");
    }
    if (require_measured_datum && !datum_measured) {
      use_gps_initialpose_ = false;
      ROS_WARN("GPS datum이 미측정 상태라 AMCL 자동 초기화를 비활성화합니다.");
    }
  }
}

// 함수이름: validate_scan
// 기능: LaserScan의 시간, lidar frame, 각도·거리 메타데이터와 NaN을 검사한다.
// 인자: message, reason
// 반환값: AMCL 입력으로 relay할 수 있으면 true
bool OdometryLidarFusion::validate_scan(const sensor_msgs::LaserScan& message,
                                        std::string* reason) const {
  if (require_laser_frame_ && message.header.frame_id != lidar_frame_) {
    *reason = "lidar_scan_frame_mismatch";
    return false;
  }
  if (!MessageValidation::validateStamp(
          message.header.stamp, ros::Time::now(), scan_max_message_age_sec_,
          max_future_stamp_sec_, have_last_scan_stamp_, last_scan_stamp_,
          reason)) {
    return false;
  }
  if (!std::isfinite(message.angle_min) ||
      !std::isfinite(message.angle_max) ||
      !std::isfinite(message.angle_increment) ||
      message.angle_max <= message.angle_min || message.angle_increment <= 0.0 ||
      !std::isfinite(message.range_min) ||
      !std::isfinite(message.range_max) || message.range_min < 0.0 ||
      message.range_max <= message.range_min ||
      message.ranges.size() < static_cast<std::size_t>(min_range_count_)) {
    *reason = "lidar_scan_metadata_invalid";
    return false;
  }
  std::size_t finite_range_count = 0;
  for (const float range : message.ranges) {
    // LaserScan에서 +/-inf는 미검출 표현으로 허용하지만 NaN은 downstream을 오염시킨다.
    if (std::isnan(range)) {
      *reason = "lidar_scan_range_nan";
      return false;
    }
    if (std::isfinite(range)) {
      ++finite_range_count;
    }
  }
  const double finite_ratio = static_cast<double>(finite_range_count) /
                              static_cast<double>(message.ranges.size());
  if (finite_ratio < min_finite_ratio_) {
    *reason = "lidar_scan_finite_ratio_too_low";
    return false;
  }
  for (const float intensity : message.intensities) {
    if (!std::isfinite(intensity)) {
      *reason = "lidar_scan_intensity_nonfinite";
      return false;
    }
  }
  reason->clear();
  return true;
}

// 함수이름: validate_map_pose
// 기능: AMCL 또는 GPS map pose의 timestamp, frame, quaternion과 covariance를 검사한다.
// 인자: message, check_monotonic_stamp, reason
// 반환값: 위치 측정값으로 사용할 수 있으면 true
bool OdometryLidarFusion::validate_map_pose(
    const geometry_msgs::PoseWithCovarianceStamped& message,
    bool check_monotonic_stamp, bool require_yaw_quality,
    std::string* reason) const {
  if (message.header.frame_id != map_frame_) {
    *reason = "map_pose_frame_mismatch";
    return false;
  }
  if (!MessageValidation::validateStamp(
          message.header.stamp, ros::Time::now(), pose_max_message_age_sec_,
          max_future_stamp_sec_, check_monotonic_stamp && have_last_pose_stamp_,
          last_pose_stamp_, reason)) {
    return false;
  }
  if (!std::isfinite(message.pose.pose.position.x) ||
      !std::isfinite(message.pose.pose.position.y) ||
      !std::isfinite(message.pose.pose.position.z)) {
    *reason = "map_pose_position_nonfinite";
    return false;
  }
  geometry_msgs::Quaternion normalized;
  if (!MessageValidation::normalizeQuaternion(
          message.pose.pose.orientation, min_quaternion_norm_,
          max_quaternion_normalization_error_, &normalized, reason)) {
    return false;
  }
  if (!MessageValidation::validateCovariance(
          message.pose.covariance.data(), 6, false,
          std::numeric_limits<double>::max(), reason)) {
    *reason = "map_pose_" + *reason;
    return false;
  }
  if (message.pose.covariance[0] > max_position_variance_m2_ ||
      message.pose.covariance[7] > max_position_variance_m2_ ||
      (require_yaw_quality &&
       message.pose.covariance[35] > max_yaw_variance_rad2_)) {
    *reason = "map_pose_covariance_too_large";
    return false;
  }
  reason->clear();
  return true;
}

// 함수이름: pose_is_stable_candidate
// 기능: 새 AMCL pose가 진행 중인 재위치 후보 주변에 연속적으로 모였는지 검사한다.
// 인자: message
// 반환값: 위치와 yaw가 안정 반경 안이면 true
bool OdometryLidarFusion::pose_is_stable_candidate(
    const geometry_msgs::PoseWithCovarianceStamped& message) const {
  if (!have_candidate_pose_) {
    return false;
  }
  const double distance_m = std::hypot(
      message.pose.pose.position.x - candidate_pose_.pose.pose.position.x,
      message.pose.pose.position.y - candidate_pose_.pose.pose.position.y);
  const double candidate_yaw = MessageValidation::yawFromQuaternion(
      candidate_pose_.pose.pose.orientation);
  const double message_yaw =
      MessageValidation::yawFromQuaternion(message.pose.pose.orientation);
  const double yaw_difference = std::abs(
      MessageValidation::shortestAngularDistance(candidate_yaw, message_yaw));
  return distance_m <= stable_candidate_radius_m_ &&
         yaw_difference <= stable_candidate_yaw_rad_;
}

// 함수이름: accept_pose_candidate
// 기능: 정상 연속 pose는 즉시, 큰 점프는 연속 안정 횟수 후 승인한다.
// 인자: message
// 반환값: 이번 pose를 Global EKF 측정으로 발행해도 되면 true
bool OdometryLidarFusion::accept_pose_candidate(
    const geometry_msgs::PoseWithCovarianceStamped& message) {
  if (have_accepted_pose_) {
    const double distance_m = std::hypot(
        message.pose.pose.position.x - accepted_pose_.pose.pose.position.x,
        message.pose.pose.position.y - accepted_pose_.pose.pose.position.y);
    const double old_yaw = MessageValidation::yawFromQuaternion(
        accepted_pose_.pose.pose.orientation);
    const double new_yaw =
        MessageValidation::yawFromQuaternion(message.pose.pose.orientation);
    const double yaw_difference = std::abs(
        MessageValidation::shortestAngularDistance(old_yaw, new_yaw));
    if (!recovery_gate_.recovering() &&
        distance_m <= max_pose_jump_m_ &&
        yaw_difference <= max_yaw_jump_rad_) {
      have_candidate_pose_ = false;
      return true;
    }
  }

  if (!pose_is_stable_candidate(message)) {
    candidate_pose_ = message;
    have_candidate_pose_ = true;
    recovery_gate_.resetCandidate();
    return recovery_gate_.observeHealthy();
  }
  candidate_pose_ = message;
  return recovery_gate_.observeHealthy();
}

void OdometryLidarFusion::publish_relocalizing(const bool relocalizing) {
  std_msgs::Bool message;
  message.data = relocalizing;
  relocalizing_publisher_.publish(message);
}

// 함수이름: scan_callback
// 기능: 검증된 LaserScan을 변경 없이 AMCL 전용 relay 토픽으로 발행한다.
// 인자: message
// 반환값: 없음
void OdometryLidarFusion::scan_callback(
    const sensor_msgs::LaserScan::ConstPtr& message) {
  std::string reason;
  if (!validate_scan(*message, &reason)) {
    ROS_WARN_THROTTLE(1.0, "Rejecting LiDAR scan: %s", reason.c_str());
    return;
  }
  scan_publisher_.publish(*message);
  last_scan_stamp_ = message->header.stamp;
  have_last_scan_stamp_ = true;
}

// 함수이름: amcl_pose_callback
// 기능: 검증과 점프 안정성 gate를 통과한 AMCL pose만 LiDAR map pose로 발행한다.
// 인자: message
// 반환값: 없음
void OdometryLidarFusion::amcl_pose_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message) {
  std::string reason;
  if (!validate_map_pose(*message, true, true, &reason)) {
    recovery_gate_.markUnhealthy();
    have_candidate_pose_ = false;
    publish_relocalizing(recovery_gate_.recovering());
    ROS_WARN_THROTTLE(1.0, "Rejecting AMCL pose: %s", reason.c_str());
    return;
  }
  if (have_last_pose_stamp_ && recovery_gate_.acceptedOnce()) {
    const double stamp_gap_sec =
        (message->header.stamp - last_pose_stamp_).toSec();
    // /clock reset 뒤의 AMCL 출력은 새로운 epoch이므로 연속 정상 횟수를 다시 센다.
    if (stamp_gap_sec < 0.0 || stamp_gap_sec > recovery_gap_sec_) {
      recovery_gate_.markUnhealthy();
      have_candidate_pose_ = false;
    }
  }
  last_pose_stamp_ = message->header.stamp;
  have_last_pose_stamp_ = true;
  if (!accept_pose_candidate(*message)) {
    publish_relocalizing(recovery_gate_.recovering());
    ROS_WARN_THROTTLE(1.0, "Waiting for stable consecutive AMCL poses");
    return;
  }
  accepted_pose_ = *message;
  MessageValidation::normalizeQuaternion(
      accepted_pose_.pose.pose.orientation, min_quaternion_norm_,
      max_quaternion_normalization_error_,
      &accepted_pose_.pose.pose.orientation, &reason);
  have_accepted_pose_ = true;
  have_candidate_pose_ = false;
  lidar_pose_publisher_.publish(accepted_pose_);
  publish_relocalizing(false);
}

// 함수이름: gps_pose_callback
// 기능: 명시적으로 활성화된 경우 최초 유효 GPS 위치와 설정 yaw를 AMCL 초기값으로 발행한다.
// 인자: message
// 반환값: 없음
void OdometryLidarFusion::gps_pose_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message) {
  if (gps_initialpose_published_ || have_accepted_pose_) {
    return;
  }
  std::string reason;
  if (!validate_map_pose(*message, false, false, &reason)) {
    ROS_WARN_THROTTLE(1.0, "Rejecting GPS initial pose: %s", reason.c_str());
    return;
  }
  geometry_msgs::PoseWithCovarianceStamped initial = *message;
  initial.pose.pose.orientation.x = 0.0;
  initial.pose.pose.orientation.y = 0.0;
  initial.pose.pose.orientation.z = std::sin(0.5 * gps_initial_yaw_rad_);
  initial.pose.pose.orientation.w = std::cos(0.5 * gps_initial_yaw_rad_);
  initial.pose.covariance[35] = gps_initial_yaw_variance_rad2_;
  initialpose_publisher_.publish(initial);
  gps_initialpose_published_ = true;
  ROS_INFO("Published one GPS-assisted AMCL initial pose");
}

}  // namespace mando_localization
