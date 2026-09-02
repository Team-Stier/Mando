#include <Arduino.h>

// Raw, motor-free RC diagnostic. This deliberately uses the same pulseIn()
// acquisition method as the user's previously working reference sketch.
// Missing pulses remain zero; no synthetic 1450 us fallback is applied.
constexpr uint8_t kSteeringPin = A0;
constexpr uint8_t kThrottlePin = A1;
constexpr uint8_t kAuxPin = A2;
constexpr unsigned long kPulseTimeoutUs = 50000UL;

void setup() {
  Serial.begin(115200);
  pinMode(kSteeringPin, INPUT);
  pinMode(kThrottlePin, INPUT);
  pinMode(kAuxPin, INPUT);
  Serial.println(F("RC_RAW_DIAGNOSTIC_READY"));
}

void loop() {
  const unsigned long steeringUs =
      pulseIn(kSteeringPin, HIGH, kPulseTimeoutUs);
  const unsigned long throttleUs =
      pulseIn(kThrottlePin, HIGH, kPulseTimeoutUs);
  const unsigned long auxUs = pulseIn(kAuxPin, HIGH, kPulseTimeoutUs);

  Serial.print(F("raw_str="));
  Serial.print(steeringUs);
  Serial.print(F(" raw_thr="));
  Serial.print(throttleUs);
  Serial.print(F(" raw_aux="));
  Serial.println(auxUs);
}
