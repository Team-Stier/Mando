#include <cstdint>

#include <gtest/gtest.h>
#include <ublox_gps/nav_pvt_time.h>

namespace {

using ublox_gps::timing::selectNavPvtTime;
using ublox_gps::timing::UtcFields;
constexpr std::int64_t kEpochNs = 1788739200000000000LL;  // 2026-09-07 UTC
constexpr std::uint64_t kReceiptNs = kEpochNs + 25000000LL;

UtcFields validUtc() {
  return UtcFields{2026, 9, 7, 0, 0, 0, 0, 7, 32};
}

TEST(NavPvtTime, MeasurementStampDoesNotAbsorbClockOffsetOrTransport) {
  const auto time = selectNavPvtTime(validUtc(), kReceiptNs, false, true);
  EXPECT_TRUE(time.utc_fields_valid);
  EXPECT_TRUE(time.utc_valid);
  EXPECT_TRUE(time.publish_fix_velocity);
  EXPECT_EQ(kEpochNs, time.gnss_utc_ns);
  EXPECT_EQ(static_cast<std::uint64_t>(kEpochNs), time.chosen_header_stamp_ns);
  EXPECT_EQ(25000000, time.receipt_minus_utc_clock_offset_plus_transport_ns);
  EXPECT_STREQ("gnss_utc", time.stamp_source);
}

TEST(NavPvtTime, NegativeNanoBorrowsAcrossUtcDay) {
  auto utc = validUtc();
  utc.nano = -250000000;
  const auto time = selectNavPvtTime(utc, kReceiptNs, false, true);
  EXPECT_TRUE(time.utc_valid);
  EXPECT_EQ(kEpochNs - 250000000, time.gnss_utc_ns);
  EXPECT_EQ(1788739199ULL, time.chosen_header_stamp_ns / 1000000000);
  EXPECT_EQ(750000000ULL, time.chosen_header_stamp_ns % 1000000000);
}

TEST(NavPvtTime, BothProtocolNanoEndpointsNormalize) {
  auto utc = validUtc();
  utc.nano = 1000000000;
  EXPECT_EQ(kEpochNs + 1000000000,
            selectNavPvtTime(utc, kReceiptNs, false, true).gnss_utc_ns);
  utc.nano = -1000000000;
  EXPECT_EQ(kEpochNs - 1000000000,
            selectNavPvtTime(utc, kReceiptNs, false, true).gnss_utc_ns);
  utc.nano = 1000000001;
  const auto high = selectNavPvtTime(utc, kReceiptNs, false, true);
  EXPECT_FALSE(high.utc_fields_valid);
  EXPECT_FALSE(high.publish_fix_velocity);
  utc.nano = -1000000001;
  EXPECT_FALSE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_fields_valid);
}

TEST(NavPvtTime, InvalidDateNeverRollsIntoAnotherMonth) {
  auto utc = validUtc();
  utc.month = 2;
  utc.day = 29;
  auto time = selectNavPvtTime(utc, kReceiptNs, false, true);
  EXPECT_FALSE(time.utc_fields_valid);
  EXPECT_STREQ("utc_day_invalid", time.utc_invalid_reason);
  utc.year = 2024;
  EXPECT_TRUE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_valid);
  utc.year = 2100;
  EXPECT_FALSE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_valid);
  utc.year = 2000;
  EXPECT_TRUE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_valid);
}

TEST(NavPvtTime, RejectsCalendarAndTimeOutsideProtocolBounds) {
  for (const int month : {0, 13}) {
    auto utc = validUtc(); utc.month = month;
    EXPECT_FALSE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_fields_valid);
  }
  for (const int day : {0, 31}) {
    auto utc = validUtc(); utc.day = day;  // September has 30 days.
    EXPECT_FALSE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_fields_valid);
  }
  auto utc = validUtc(); utc.hour = 24;
  EXPECT_FALSE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_fields_valid);
  utc = validUtc(); utc.minute = 60;
  EXPECT_FALSE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_fields_valid);
  utc = validUtc(); utc.second = 61;
  EXPECT_FALSE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_fields_valid);
}

TEST(NavPvtTime, LeapSecondUsesPosixNormalization) {
  auto utc = validUtc();
  utc.year = 2016; utc.month = 12; utc.day = 31;
  utc.hour = 23; utc.minute = 59; utc.second = 60;
  const auto time = selectNavPvtTime(utc, kReceiptNs, false, true);
  EXPECT_TRUE(time.utc_valid);
  EXPECT_EQ(1483228800000000000LL, time.gnss_utc_ns);
}

TEST(NavPvtTime, MissingUtcFlagsRejectsMeasurementButPreservesCandidate) {
  for (const std::uint8_t missing : {1, 2, 4}) {
    auto utc = validUtc(); utc.valid_flags &= ~missing;
    const auto time = selectNavPvtTime(utc, kReceiptNs, false, true);
    EXPECT_TRUE(time.utc_fields_valid);
    EXPECT_FALSE(time.utc_valid);
    EXPECT_FALSE(time.publish_fix_velocity);
    EXPECT_EQ(kEpochNs, time.gnss_utc_ns);
    EXPECT_STREQ("invalid_utc_rejected", time.stamp_source);
  }
}

TEST(NavPvtTime, ExistingConfirmationAvailabilityPolicyIsExplicit) {
  auto utc = validUtc(); utc.confirmation_flags = 0;
  auto time = selectNavPvtTime(utc, kReceiptNs, false, true);
  EXPECT_FALSE(time.utc_valid);
  EXPECT_STREQ("utc_confirmation_unavailable", time.utc_invalid_reason);
  utc.confirmation_flags = 32;
  EXPECT_TRUE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_valid);
}

TEST(NavPvtTime, LegacyInvalidUtcFallbackRemainsOptOutDefault) {
  auto utc = validUtc(); utc.valid_flags = 0;
  const auto time = selectNavPvtTime(utc, kReceiptNs, false, false);
  EXPECT_FALSE(time.utc_valid);
  EXPECT_TRUE(time.publish_fix_velocity);
  EXPECT_EQ(kReceiptNs, time.chosen_header_stamp_ns);
  EXPECT_STREQ("ros_receipt_invalid_utc_fallback", time.stamp_source);
}

TEST(NavPvtTime, ExplicitReceiptModeNeverPretendsToBeMeasurementTime) {
  auto utc = validUtc(); utc.valid_flags = 0;
  const auto time = selectNavPvtTime(utc, kReceiptNs, true, true);
  EXPECT_FALSE(time.utc_valid);
  EXPECT_TRUE(time.publish_fix_velocity);
  EXPECT_EQ(kReceiptNs, time.chosen_header_stamp_ns);
  EXPECT_STREQ("ros_receipt_explicit", time.stamp_source);
}

TEST(NavPvtTime, ReceiptEarlierThanUtcReportsSignedDifference) {
  const auto time = selectNavPvtTime(validUtc(), kEpochNs - 1200000000,
                                   false, true);
  EXPECT_EQ(-1200000000,
            time.receipt_minus_utc_clock_offset_plus_transport_ns);
  EXPECT_EQ(static_cast<std::uint64_t>(kEpochNs), time.chosen_header_stamp_ns);
}

TEST(NavPvtTime, RejectsUnderflowAndRosOverflow) {
  auto utc = validUtc();
  utc.year = 1970; utc.month = 1; utc.day = 1; utc.nano = -1;
  EXPECT_FALSE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_fields_valid);
  utc = validUtc();
  utc.year = 2106; utc.month = 2; utc.day = 7;
  utc.hour = 6; utc.minute = 28; utc.second = 15; utc.nano = 999999999;
  const auto last = selectNavPvtTime(utc, kReceiptNs, false, true);
  EXPECT_TRUE(last.utc_valid);
  EXPECT_EQ(ublox_gps::timing::kMaximumRosStampNs, last.gnss_utc_ns);
  utc.nano = 1000000000;
  EXPECT_FALSE(selectNavPvtTime(utc, kReceiptNs, false, true).utc_fields_valid);
}

}  // namespace
