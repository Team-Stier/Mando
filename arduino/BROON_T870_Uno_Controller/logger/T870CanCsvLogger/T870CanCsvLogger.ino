#include <Arduino.h>

#include "T870CanProtocol.h"
#include "T870Mcp2515.h"

namespace {

constexpr uint8_t kCanChipSelectPin = 10U;
constexpr uint32_t kRetryIntervalMs = 1000UL;
constexpr uint8_t kAllFramesSeenMask =
    static_cast<uint8_t>((1U << T870Can::kFrameTypeCount) - 1U);

T870Can::Mcp2515 canController(kCanChipSelectPin);
bool canReady = false;
uint32_t lastRetryMs = 0UL;

struct DecodedTelemetry {
  uint8_t sequence;
  uint8_t protocolVersion;
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
  uint8_t encoderCorrected;

  uint16_t rcSteerUs;
  uint16_t rcThrottleUs;
  uint16_t rcAuxUs;
  uint8_t rcFlags;

  uint16_t rcReadUs;
  uint16_t droppedFrames;
  uint8_t senderErrorFlags;
  uint8_t senderTxErrorCount;
  uint8_t senderRxErrorCount;
};

DecodedTelemetry decoded = {};
bool sequenceStarted = false;
uint8_t seenMask = 0U;

void printCsvHeader() {
  Serial.println(F(
      "logger_ms,complete,seq,protocol,state,fault,mode,status_flags,"
      "uptime_s,drive_req_pwm,front_pwm,rear_pwm,steer_target_adc,"
      "steer_actual_adc,steer_pwm,speed_kph,encoder_delta,"
      "encoder_corrected,rc_steer_us,rc_throttle_us,rc_aux_us,rc_flags,"
      "rc_read_us,tx_dropped,can_eflg,can_tec,can_rec"));
}

void printHexByte(uint8_t value) {
  if (value < 0x10U) Serial.print('0');
  Serial.print(value, HEX);
}

void printRawCanFrame(uint32_t receivedAtMs, const T870Can::Frame& frame) {
  // Prefixing raw frames keeps them unambiguous when they share the same USB
  // serial stream as the decoded telemetry rows. The PC capture program saves
  // these lines separately without changing the existing diagnosis CSV.
  Serial.print(F("@CAN,"));
  Serial.print(receivedAtMs);
  Serial.print(F(",0x"));
  if (frame.id < 0x100U) Serial.print('0');
  if (frame.id < 0x010U) Serial.print('0');
  Serial.print(frame.id, HEX);
  Serial.print(',');
  Serial.print(frame.length);
  Serial.print(',');
  for (uint8_t index = 0U; index < frame.length; ++index) {
    printHexByte(frame.data[index]);
  }
  Serial.println();
}

void startSequence(uint8_t sequence) {
  if (sequenceStarted && decoded.sequence == sequence) return;
  decoded = DecodedTelemetry();
  decoded.sequence = sequence;
  sequenceStarted = true;
  seenMask = 0U;
}

void printCsvRow() {
  const bool complete = seenMask == kAllFramesSeenMask;
  Serial.print(millis());
  Serial.print(',');
  Serial.print(complete ? 1 : 0);
  Serial.print(',');
  Serial.print(decoded.sequence);
  Serial.print(',');
  Serial.print(decoded.protocolVersion);
  Serial.print(',');
  Serial.print(decoded.state);
  Serial.print(',');
  Serial.print(decoded.fault);
  Serial.print(',');
  Serial.print(decoded.mode);
  Serial.print(',');
  Serial.print(decoded.statusFlags);
  Serial.print(',');
  Serial.print(decoded.uptimeSeconds);
  Serial.print(',');
  Serial.print(decoded.requestedDrivePwm);
  Serial.print(',');
  Serial.print(decoded.frontDrivePwm);
  Serial.print(',');
  Serial.print(decoded.rearDrivePwm);
  Serial.print(',');
  Serial.print(decoded.steeringTargetAdc);
  Serial.print(',');
  Serial.print(decoded.steeringActualAdc);
  Serial.print(',');
  Serial.print(decoded.steeringPwm);
  Serial.print(',');
  Serial.print(static_cast<float>(decoded.speedCentiKph) / 100.0f, 2);
  Serial.print(',');
  Serial.print(decoded.encoderDelta);
  Serial.print(',');
  Serial.print(decoded.encoderCorrected);
  Serial.print(',');
  Serial.print(decoded.rcSteerUs);
  Serial.print(',');
  Serial.print(decoded.rcThrottleUs);
  Serial.print(',');
  Serial.print(decoded.rcAuxUs);
  Serial.print(',');
  Serial.print(decoded.rcFlags);
  Serial.print(',');
  Serial.print(decoded.rcReadUs);
  Serial.print(',');
  Serial.print(decoded.droppedFrames);
  Serial.print(',');
  Serial.print(decoded.senderErrorFlags);
  Serial.print(',');
  Serial.print(decoded.senderTxErrorCount);
  Serial.print(',');
  Serial.println(decoded.senderRxErrorCount);
}

void consumeFrame(const T870Can::Frame& frame) {
  if (!T870Can::isTelemetryFrame(frame)) return;
  const uint8_t sequence = frame.data[0];
  startSequence(sequence);

  switch (frame.id) {
    case T870Can::kStatusFrameId:
      decoded.state = frame.data[1];
      decoded.fault = frame.data[2];
      decoded.mode = frame.data[3];
      decoded.statusFlags = frame.data[4];
      decoded.uptimeSeconds = T870Can::readU16(&frame.data[5]);
      decoded.protocolVersion = frame.data[7];
      seenMask |= 1U << 0;
      break;
    case T870Can::kDriveFrameId:
      decoded.requestedDrivePwm = T870Can::readI16(&frame.data[1]);
      decoded.frontDrivePwm = T870Can::readI16(&frame.data[3]);
      decoded.rearDrivePwm = T870Can::readI16(&frame.data[5]);
      seenMask |= 1U << 1;
      break;
    case T870Can::kSteeringFrameId:
      decoded.steeringTargetAdc = T870Can::readU16(&frame.data[1]);
      decoded.steeringActualAdc = T870Can::readU16(&frame.data[3]);
      decoded.steeringPwm = T870Can::readI16(&frame.data[5]);
      seenMask |= 1U << 2;
      break;
    case T870Can::kMotionFrameId:
      decoded.speedCentiKph = T870Can::readI16(&frame.data[1]);
      decoded.encoderDelta = T870Can::readI32(&frame.data[3]);
      decoded.encoderCorrected = frame.data[7];
      seenMask |= 1U << 3;
      break;
    case T870Can::kRcFrameId:
      decoded.rcSteerUs = T870Can::readU16(&frame.data[1]);
      decoded.rcThrottleUs = T870Can::readU16(&frame.data[3]);
      decoded.rcAuxUs = T870Can::readU16(&frame.data[5]);
      decoded.rcFlags = frame.data[7];
      seenMask |= 1U << 4;
      break;
    case T870Can::kTimingFrameId:
      decoded.rcReadUs = T870Can::readU16(&frame.data[1]);
      decoded.droppedFrames = T870Can::readU16(&frame.data[3]);
      decoded.senderErrorFlags = frame.data[5];
      decoded.senderTxErrorCount = frame.data[6];
      decoded.senderRxErrorCount = frame.data[7];
      seenMask |= 1U << 5;
      printCsvRow();
      break;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  printCsvHeader();
  canReady = canController.begin500K8MHz();
  if (!canReady) {
    Serial.println(F("# MCP2515_INIT_FAILED; retrying every 1000 ms"));
  }
}

void loop() {
  const uint32_t nowMs = millis();
  if (!canReady) {
    if (static_cast<uint32_t>(nowMs - lastRetryMs) >= kRetryIntervalMs) {
      lastRetryMs = nowMs;
      canReady = canController.begin500K8MHz();
      if (canReady) Serial.println(F("# MCP2515_READY"));
    }
    return;
  }

  T870Can::Frame frame;
  while (canController.tryReceive(frame)) {
    printRawCanFrame(millis(), frame);
    consumeFrame(frame);
  }
}
