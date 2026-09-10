#ifndef UBLOX_GPS_NAV_PVT_TIME_H
#define UBLOX_GPS_NAV_PVT_TIME_H

#include <cstdint>

namespace ublox_gps {
namespace timing {

// NAV-PVT protocol constants; both firmware 7 and later use these UTC fields.
constexpr std::uint8_t kRequiredUtcValidFlags = 1 | 2 | 4;
constexpr std::uint8_t kUtcConfirmationAvailable = 32;
constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr std::int64_t kMaximumRosStampNs = 4294967295999999999LL;

struct UtcFields {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
  std::int64_t nano;
  std::uint8_t valid_flags;
  std::uint8_t confirmation_flags;
};

struct TimingDecision {
  bool utc_fields_valid = false;
  bool utc_valid = false;
  bool publish_fix_velocity = false;
  std::int64_t gnss_utc_ns = 0;
  std::uint64_t callback_ros_receipt_ns = 0;
  std::uint64_t chosen_header_stamp_ns = 0;
  std::int64_t receipt_minus_utc_clock_offset_plus_transport_ns = 0;
  const char* stamp_source = "invalid_utc_rejected";
  const char* utc_invalid_reason = "";
};

inline bool isLeapYear(const int year) {
  return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

// Calendar validation precedes conversion: invalid dates must not silently
// normalize into a different measurement date. POSIX/ROS represents sec=60 as
// the following second, so the original second is also exposed in diagnostics.
inline const char* utcNanoseconds(const UtcFields& utc, std::int64_t* output) {
  if (utc.year < 1970 || utc.year > 2106) {
    return "utc_year_out_of_ros_range";
  }
  if (utc.month < 1 || utc.month > 12) {
    return "utc_month_invalid";
  }
  const int month_days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  const int last_day = month_days[utc.month - 1] +
      (utc.month == 2 && isLeapYear(utc.year) ? 1 : 0);
  if (utc.day < 1 || utc.day > last_day) {
    return "utc_day_invalid";
  }
  if (utc.hour < 0 || utc.hour > 23 || utc.minute < 0 ||
      utc.minute > 59 || utc.second < 0 || utc.second > 60) {
    return "utc_time_of_day_invalid";
  }
  if (utc.nano < -kNanosecondsPerSecond || utc.nano > kNanosecondsPerSecond) {
    return "utc_nanosecond_out_of_range";
  }
  std::int64_t days = utc.day - 1;
  for (int year = 1970; year < utc.year; ++year) {
    days += isLeapYear(year) ? 366 : 365;
  }
  for (int month = 1; month < utc.month; ++month) {
    days += month_days[month - 1] +
        (month == 2 && isLeapYear(utc.year) ? 1 : 0);
  }
  const std::int64_t seconds =
      ((days * 24 + utc.hour) * 60 + utc.minute) * 60 + utc.second;
  const std::int64_t nanoseconds = seconds * kNanosecondsPerSecond + utc.nano;
  if (nanoseconds < 0 || nanoseconds > kMaximumRosStampNs) {
    return "utc_timestamp_out_of_ros_range";
  }
  *output = nanoseconds;
  return "";
}

inline TimingDecision selectNavPvtTime(const UtcFields& utc,
                                      const std::uint64_t receipt_ns,
                                      const bool use_ros_time,
                                      const bool require_valid_utc) {
  TimingDecision result;
  result.callback_ros_receipt_ns = receipt_ns;
  result.utc_invalid_reason = utcNanoseconds(utc, &result.gnss_utc_ns);
  result.utc_fields_valid = result.utc_invalid_reason[0] == '\0';
  if (result.utc_fields_valid) {
    result.receipt_minus_utc_clock_offset_plus_transport_ns =
        static_cast<std::int64_t>(receipt_ns) - result.gnss_utc_ns;
    // Preserve the driver's existing NAV-PVT validity policy. Confirmation
    // flags are separately reported; tAcc is not PC clock synchronization.
    if ((utc.valid_flags & kRequiredUtcValidFlags) != kRequiredUtcValidFlags) {
      result.utc_invalid_reason = "utc_date_time_not_fully_resolved";
    } else if (!(utc.confirmation_flags & kUtcConfirmationAvailable)) {
      result.utc_invalid_reason = "utc_confirmation_unavailable";
    } else {
      result.utc_valid = true;
    }
  }
  if (use_ros_time) {
    result.publish_fix_velocity = true;
    result.chosen_header_stamp_ns = receipt_ns;
    result.stamp_source = "ros_receipt_explicit";
  } else if (result.utc_valid) {
    result.publish_fix_velocity = true;
    result.chosen_header_stamp_ns = static_cast<std::uint64_t>(result.gnss_utc_ns);
    result.stamp_source = "gnss_utc";
  } else if (!require_valid_utc) {
    result.publish_fix_velocity = true;
    result.chosen_header_stamp_ns = receipt_ns;
    result.stamp_source = "ros_receipt_invalid_utc_fallback";
  }
  return result;
}

}  // namespace timing
}  // namespace ublox_gps

#endif  // UBLOX_GPS_NAV_PVT_TIME_H
