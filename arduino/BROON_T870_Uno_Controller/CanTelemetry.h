#pragma once

#include <Arduino.h>

#include "BuildOptions.h"
#include "T870CanProtocol.h"

#if BROON_ENABLE_CAN_TELEMETRY
#include "T870Mcp2515.h"
#endif

namespace BroonT870Controller {

class CanTelemetry {
 public:
  CanTelemetry();

  // Initialization failure is diagnostic only. It never authorizes or blocks
  // vehicle outputs and it is never retried from the control loop.
  bool begin();
  void update(uint32_t nowMs, const T870Can::TelemetrySnapshot& snapshot);

  bool initialized() const;
  uint16_t droppedFrames() const;

 private:
#if BROON_ENABLE_CAN_TELEMETRY
  T870Can::Mcp2515 controller_;
  bool initialized_;
  bool clockStarted_;
  uint32_t lastFrameMs_;
  uint8_t frameSlot_;
  uint8_t sequence_;
  uint16_t droppedFrames_;
#endif
};

}  // namespace BroonT870Controller
