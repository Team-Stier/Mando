#include <Arduino.h>

#include "T870CanProtocol.h"

namespace {

uint16_t passed = 0U;
uint16_t failed = 0U;

void expect(bool condition, const __FlashStringHelper* label) {
  if (condition) {
    ++passed;
    Serial.print(F("PASS "));
  } else {
    ++failed;
    Serial.print(F("FAIL "));
  }
  Serial.println(label);
}

void runTests() {
  const T870Can::TelemetrySnapshot snapshot = {
      2U,    6U,    1U,    0x5BU, 4321U, -80, 75,   -76,
      1020U, 600U,  -170,  234,   -123456L, true, 1100U, 1450U,
      1800U, 0x0FU, 65535U};

  const T870Can::Frame status = T870Can::makeStatusFrame(snapshot, 42U);
  expect(status.id == 0x100U && status.length == 8U, F("status identity"));
  expect(status.data[0] == 42U && status.data[1] == 2U &&
             status.data[2] == 6U && status.data[3] == 1U &&
             status.data[4] == 0x5BU,
         F("status fields"));
  expect(T870Can::readU16(&status.data[5]) == 4321U &&
             status.data[7] == T870Can::kProtocolVersion,
         F("status uptime/version"));

  const T870Can::Frame drive = T870Can::makeDriveFrame(snapshot, 42U);
  expect(T870Can::readI16(&drive.data[1]) == -80 &&
             T870Can::readI16(&drive.data[3]) == 75 &&
             T870Can::readI16(&drive.data[5]) == -76,
         F("drive signed fields"));

  const T870Can::Frame steering =
      T870Can::makeSteeringFrame(snapshot, 42U);
  expect(T870Can::readU16(&steering.data[1]) == 1020U &&
             T870Can::readU16(&steering.data[3]) == 600U &&
             T870Can::readI16(&steering.data[5]) == -170,
         F("steering fields"));

  const T870Can::Frame motion = T870Can::makeMotionFrame(snapshot, 42U);
  expect(T870Can::readI16(&motion.data[1]) == 234 &&
             T870Can::readI32(&motion.data[3]) == -123456L &&
             motion.data[7] == 1U,
         F("motion fields"));

  const T870Can::Frame rc = T870Can::makeRcFrame(snapshot, 42U);
  expect(T870Can::readU16(&rc.data[1]) == 1100U &&
             T870Can::readU16(&rc.data[3]) == 1450U &&
             T870Can::readU16(&rc.data[5]) == 1800U &&
             rc.data[7] == 0x0FU,
         F("RC fields"));

  const T870Can::Frame timing =
      T870Can::makeTimingFrame(snapshot, 42U, 513U, 0x20U, 128U, 7U);
  expect(T870Can::readU16(&timing.data[1]) == 65535U &&
             T870Can::readU16(&timing.data[3]) == 513U &&
             timing.data[5] == 0x20U && timing.data[6] == 128U &&
             timing.data[7] == 7U,
         F("timing fields"));
  expect(T870Can::isTelemetryFrame(timing), F("telemetry recognition"));

  Serial.print(F("TOTAL pass="));
  Serial.print(passed);
  Serial.print(F(" fail="));
  Serial.println(failed);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  runTests();
}

void loop() {}
