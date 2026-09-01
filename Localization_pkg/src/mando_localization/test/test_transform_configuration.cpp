#include <gtest/gtest.h>

#include <cmath>

#include "../src/tf/transform_configuration.hpp"

namespace mando_localization {
namespace {

StaticTransformSpec transform(const std::string& parent, const std::string& child) {
  StaticTransformSpec spec;
  spec.parent_frame = parent;
  spec.child_frame = child;
  spec.calibration_state = "unmeasured";
  return spec;
}

TEST(TransformConfigurationTest, DoesNotPublishUnmeasuredTransform) {
  StaticTransformSpec spec = transform("base_link", "imu_link");
  spec.enabled = true;
  EXPECT_FALSE(TransformConfiguration::shouldPublish(spec));
  spec.calibration_state = "measured";
  EXPECT_TRUE(TransformConfiguration::shouldPublish(spec));
}

TEST(TransformConfigurationTest, RejectsDuplicateChild) {
  EXPECT_THROW(TransformConfiguration::validate(
                   {transform("base_link", "imu_link"), transform("odom", "imu_link")}),
               std::runtime_error);
}

TEST(TransformConfigurationTest, RejectsCycle) {
  EXPECT_THROW(TransformConfiguration::validate(
                   {transform("base_link", "imu_link"), transform("imu_link", "base_link")}),
               std::runtime_error);
}

TEST(TransformConfigurationTest, RejectsDuplicateChildAcrossDynamicAndStatic) {
  DynamicTransformSpec dynamic;
  dynamic.parent_frame = "odom";
  dynamic.child_frame = "base_link";
  dynamic.owner_node = "local_ekf";
  dynamic.source = "robot_localization";
  EXPECT_THROW(TransformConfiguration::validateTree(
                   {transform("map", "base_link")}, {dynamic}),
               std::runtime_error);
}

TEST(TransformConfigurationTest, ConvertsDegreesToNormalizedQuaternion) {
  StaticTransformSpec spec = transform("base_link", "gps_link");
  spec.yaw_deg = 90.0;
  const geometry_msgs::TransformStamped message =
      TransformConfiguration::toMessage(spec, ros::Time(1.0));
  EXPECT_NEAR(std::sqrt(0.5), message.transform.rotation.z, 1.0e-9);
  EXPECT_NEAR(std::sqrt(0.5), message.transform.rotation.w, 1.0e-9);
}

}  // namespace
}  // namespace mando_localization
