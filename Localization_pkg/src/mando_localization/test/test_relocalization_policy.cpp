#include <gtest/gtest.h>

#include "../src/relocalization/relocalization_policy.hpp"

namespace mando_localization {
namespace {

RelocalizationPolicy policy(const bool gps_only_enabled = false,
                            const bool datum_measured = true) {
  RelocalizationPolicyConfig config;
  config.max_gps_lidar_distance_m = 5.0;
  config.max_stationary_speed_mps = 0.3;
  config.lidar_required_consecutive_candidates = 3;
  config.gps_only_required_consecutive_candidates = 5;
  config.automatic_gps_only_reset_enabled = gps_only_enabled;
  config.gps_datum_measured = datum_measured;
  config.require_measured_datum = true;
  return RelocalizationPolicy(config);
}

RelocalizationObservation longOutage() {
  RelocalizationObservation observation;
  observation.have_prior_anchor = true;
  observation.long_outage = true;
  return observation;
}

TEST(RelocalizationPolicyTest, NormalTrackingDoesNotEnterRecovery) {
  RelocalizationObservation observation = longOutage();
  observation.long_outage = false;
  const auto decision = policy().evaluate(observation);
  EXPECT_EQ(RecoveryAction::NONE, decision.action);
  EXPECT_FALSE(decision.relocalizing);
}

TEST(RelocalizationPolicyTest, LidarAgreementOverridesDriftedLocalPrediction) {
  RelocalizationObservation observation = longOutage();
  observation.lidar_healthy = true;
  observation.gps_lidar_distance_m = 1.0;
  observation.consecutive_candidates = 3;
  const auto decision = policy().evaluate(observation);
  EXPECT_EQ(RecoveryAction::ACCEPT_WITH_LIDAR, decision.action);
  EXPECT_EQ("LIDAR_RECOVERY_READY", decision.state);
}

TEST(RelocalizationPolicyTest, LidarMismatchIsFailClosed) {
  RelocalizationObservation observation = longOutage();
  observation.lidar_healthy = true;
  observation.gps_lidar_distance_m = 5.01;
  observation.consecutive_candidates = 10;
  const auto decision = policy().evaluate(observation);
  EXPECT_EQ(RecoveryAction::NONE, decision.action);
  EXPECT_TRUE(decision.relocalizing);
  EXPECT_EQ("LIDAR_MISMATCH", decision.state);
}

TEST(RelocalizationPolicyTest, GpsOnlyResetIsDisabledByDefault) {
  RelocalizationObservation observation = longOutage();
  observation.twist_healthy = true;
  observation.global_odometry_healthy = true;
  observation.consecutive_candidates = 5;
  const auto decision = policy().evaluate(observation);
  EXPECT_EQ(RecoveryAction::NONE, decision.action);
  EXPECT_EQ("GPS_ONLY_BLOCKED", decision.state);
}

TEST(RelocalizationPolicyTest, GpsOnlyResetRequiresStationaryStableCandidates) {
  RelocalizationObservation observation = longOutage();
  observation.twist_healthy = true;
  observation.global_odometry_healthy = true;
  observation.speed_mps = 0.31;
  observation.consecutive_candidates = 5;
  EXPECT_EQ("WAITING_FOR_STATIONARY", policy(true).evaluate(observation).state);

  observation.speed_mps = 0.1;
  observation.consecutive_candidates = 4;
  EXPECT_EQ("VALIDATING_GPS_ONLY", policy(true).evaluate(observation).state);

  observation.consecutive_candidates = 5;
  EXPECT_EQ(RecoveryAction::RESET_WITH_GPS,
            policy(true).evaluate(observation).action);
}

TEST(RelocalizationPolicyTest, GpsOnlyResetRequiresMeasuredDatum) {
  RelocalizationObservation observation = longOutage();
  observation.twist_healthy = true;
  observation.global_odometry_healthy = true;
  observation.speed_mps = 0.0;
  observation.consecutive_candidates = 5;
  EXPECT_EQ("GPS_ONLY_BLOCKED_DATUM",
            policy(true, false).evaluate(observation).state);
}

}  // namespace
}  // namespace mando_localization
