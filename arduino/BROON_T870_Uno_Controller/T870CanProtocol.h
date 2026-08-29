#pragma once

#include <stdint.h>

namespace T870Can {

constexpr uint8_t kProtocolVersion = 1U;
constexpr uint8_t kFrameLength = 8U;

constexpr uint16_t kStatusFrameId = 0x100U;
constexpr uint16_t kDriveFrameId = 0x101U;
constexpr uint16_t kSteeringFrameId = 0x102U;
constexpr uint16_t kMotionFrameId = 0x103U;
constexpr uint16_t kRcFrameId = 0x104U;
constexpr uint16_t kTimingFrameId = 0x105U;
constexpr uint8_t kFrameTypeCount = 6U;

enum StatusFlag : uint8_t {
  STATUS_OUTPUTS_ALLOWED = 1U << 0,
  STATUS_IMMEDIATE_STOP = 1U << 1,
  STATUS_RC_STOP = 1U << 2,
  STATUS_COMMAND_VALID = 1U << 3,
  STATUS_CONFIGURATION_VALID = 1U << 4,
  STATUS_STEERING_FAULT = 1U << 5,
  STATUS_DRIVE_FAULT = 1U << 6
};

enum RcFlag : uint8_t {
  RC_STEER_VALID = 1U << 0,
  RC_THROTTLE_VALID = 1U << 1,
  RC_AUX_VALID = 1U << 2,
  RC_REMOTE_STOP = 1U << 3
};

struct Frame {
  uint16_t id;
  uint8_t length;
  uint8_t data[kFrameLength];
};

struct TelemetrySnapshot {
  uint8_t state;
  uint8_t fault;
  uint8_t mode;
  uint8_t statusFlags;
  uint16_t uptimeSeconds;

  int16_t requestedDrivePwm;
  int16_t frontDrivePwm;
  int16_t rearDrivePwm;

  uint16_t steeringTargetAdc;
  uint16_t steeringActualAdc;
  int16_t steeringPwm;

  int16_t speedCentiKph;
  int32_t encoderDelta;
  bool encoderCorrected;

  uint16_t rcSteerUs;
  uint16_t rcThrottleUs;
  uint16_t rcAuxUs;
  uint8_t rcFlags;

  uint16_t rcReadUs;
};

inline void clearFrame(Frame& frame, uint16_t id) {
  frame.id = id;
  frame.length = kFrameLength;
  for (uint8_t index = 0U; index < kFrameLength; ++index) {
    frame.data[index] = 0U;
  }
}

inline void writeU16(uint8_t* destination, uint16_t value) {
  destination[0] = static_cast<uint8_t>(value & 0xFFU);
  destination[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}

inline void writeI16(uint8_t* destination, int16_t value) {
  writeU16(destination, static_cast<uint16_t>(value));
}

inline void writeI32(uint8_t* destination, int32_t value) {
  const uint32_t bits = static_cast<uint32_t>(value);
  destination[0] = static_cast<uint8_t>(bits & 0xFFUL);
  destination[1] = static_cast<uint8_t>((bits >> 8) & 0xFFUL);
  destination[2] = static_cast<uint8_t>((bits >> 16) & 0xFFUL);
  destination[3] = static_cast<uint8_t>((bits >> 24) & 0xFFUL);
}

inline uint16_t readU16(const uint8_t* source) {
  return static_cast<uint16_t>(source[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(source[1]) << 8);
}

inline int16_t readI16(const uint8_t* source) {
  return static_cast<int16_t>(readU16(source));
}

inline int32_t readI32(const uint8_t* source) {
  const uint32_t bits = static_cast<uint32_t>(source[0]) |
                        (static_cast<uint32_t>(source[1]) << 8) |
                        (static_cast<uint32_t>(source[2]) << 16) |
                        (static_cast<uint32_t>(source[3]) << 24);
  return static_cast<int32_t>(bits);
}

inline Frame makeStatusFrame(const TelemetrySnapshot& snapshot,
                             uint8_t sequence) {
  Frame frame;
  clearFrame(frame, kStatusFrameId);
  frame.data[0] = sequence;
  frame.data[1] = snapshot.state;
  frame.data[2] = snapshot.fault;
  frame.data[3] = snapshot.mode;
  frame.data[4] = snapshot.statusFlags;
  writeU16(&frame.data[5], snapshot.uptimeSeconds);
  frame.data[7] = kProtocolVersion;
  return frame;
}

inline Frame makeDriveFrame(const TelemetrySnapshot& snapshot,
                            uint8_t sequence) {
  Frame frame;
  clearFrame(frame, kDriveFrameId);
  frame.data[0] = sequence;
  writeI16(&frame.data[1], snapshot.requestedDrivePwm);
  writeI16(&frame.data[3], snapshot.frontDrivePwm);
  writeI16(&frame.data[5], snapshot.rearDrivePwm);
  return frame;
}

inline Frame makeSteeringFrame(const TelemetrySnapshot& snapshot,
                               uint8_t sequence) {
  Frame frame;
  clearFrame(frame, kSteeringFrameId);
  frame.data[0] = sequence;
  writeU16(&frame.data[1], snapshot.steeringTargetAdc);
  writeU16(&frame.data[3], snapshot.steeringActualAdc);
  writeI16(&frame.data[5], snapshot.steeringPwm);
  return frame;
}

inline Frame makeMotionFrame(const TelemetrySnapshot& snapshot,
                             uint8_t sequence) {
  Frame frame;
  clearFrame(frame, kMotionFrameId);
  frame.data[0] = sequence;
  writeI16(&frame.data[1], snapshot.speedCentiKph);
  writeI32(&frame.data[3], snapshot.encoderDelta);
  frame.data[7] = snapshot.encoderCorrected ? 1U : 0U;
  return frame;
}

inline Frame makeRcFrame(const TelemetrySnapshot& snapshot,
                         uint8_t sequence) {
  Frame frame;
  clearFrame(frame, kRcFrameId);
  frame.data[0] = sequence;
  writeU16(&frame.data[1], snapshot.rcSteerUs);
  writeU16(&frame.data[3], snapshot.rcThrottleUs);
  writeU16(&frame.data[5], snapshot.rcAuxUs);
  frame.data[7] = snapshot.rcFlags;
  return frame;
}

inline Frame makeTimingFrame(const TelemetrySnapshot& snapshot,
                             uint8_t sequence, uint16_t droppedFrames,
                             uint8_t errorFlags, uint8_t txErrorCount,
                             uint8_t rxErrorCount) {
  Frame frame;
  clearFrame(frame, kTimingFrameId);
  frame.data[0] = sequence;
  writeU16(&frame.data[1], snapshot.rcReadUs);
  writeU16(&frame.data[3], droppedFrames);
  frame.data[5] = errorFlags;
  frame.data[6] = txErrorCount;
  frame.data[7] = rxErrorCount;
  return frame;
}

inline bool isTelemetryFrame(const Frame& frame) {
  return frame.length == kFrameLength && frame.id >= kStatusFrameId &&
         frame.id <= kTimingFrameId;
}

}  // namespace T870Can
