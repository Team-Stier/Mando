#include "CanTelemetry.h"

#include "BroonT870Core.h"
#include "ControllerConfig.h"

namespace BroonT870Controller {

CanTelemetry::CanTelemetry()
#if BROON_ENABLE_CAN_TELEMETRY
    : controller_(kCanChipSelectPin),
      initialized_(false),
      clockStarted_(false),
      lastFrameMs_(0UL),
      frameSlot_(0U),
      sequence_(0U),
      droppedFrames_(0U)
#endif
{
}

bool CanTelemetry::begin() {
#if BROON_ENABLE_CAN_TELEMETRY
  initialized_ = controller_.begin500K8MHz();
  clockStarted_ = false;
  frameSlot_ = 0U;
  sequence_ = 0U;
  droppedFrames_ = 0U;
  return initialized_;
#else
  return false;
#endif
}

void CanTelemetry::update(uint32_t nowMs,
                          const T870Can::TelemetrySnapshot& snapshot) {
#if BROON_ENABLE_CAN_TELEMETRY
  if (!initialized_) return;
  if (!clockStarted_) {
    lastFrameMs_ = nowMs;
    clockStarted_ = true;
    return;
  }
  if (!BroonT870::hasElapsed(nowMs, lastFrameMs_, kCanFrameIntervalMs)) {
    return;
  }
  lastFrameMs_ = nowMs;

  T870Can::Frame frame;
  switch (frameSlot_) {
    case 0U:
      frame = T870Can::makeStatusFrame(snapshot, sequence_);
      break;
    case 1U:
      frame = T870Can::makeDriveFrame(snapshot, sequence_);
      break;
    case 2U:
      frame = T870Can::makeSteeringFrame(snapshot, sequence_);
      break;
    case 3U:
      frame = T870Can::makeMotionFrame(snapshot, sequence_);
      break;
    case 4U:
      frame = T870Can::makeRcFrame(snapshot, sequence_);
      break;
    default:
      frame = T870Can::makeTimingFrame(
          snapshot, sequence_, droppedFrames_, controller_.errorFlags(),
          controller_.transmitErrorCount(), controller_.receiveErrorCount());
      break;
  }

  if (!controller_.trySend(frame) && droppedFrames_ < 0xFFFFU) {
    ++droppedFrames_;
  }

  ++frameSlot_;
  if (frameSlot_ >= T870Can::kFrameTypeCount) {
    frameSlot_ = 0U;
    ++sequence_;
  }
#else
  (void)nowMs;
  (void)snapshot;
#endif
}

bool CanTelemetry::initialized() const {
#if BROON_ENABLE_CAN_TELEMETRY
  return initialized_;
#else
  return false;
#endif
}

uint16_t CanTelemetry::droppedFrames() const {
#if BROON_ENABLE_CAN_TELEMETRY
  return droppedFrames_;
#else
  return 0U;
#endif
}

}  // namespace BroonT870Controller
