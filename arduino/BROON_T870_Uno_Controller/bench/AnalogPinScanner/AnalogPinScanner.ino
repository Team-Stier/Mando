// Mega analog-input diagnostic. Keep all motor power disconnected.
// Upload this sketch separately, then open Serial Monitor at 115200 baud.

constexpr uint8_t kAnalogPins[] = {
    A0, A1, A2, A3, A4, A5, A6, A7,
    A8, A9, A10, A11, A12, A13, A14, A15};

void setup() {
  Serial.begin(115200);
  analogReference(DEFAULT);
  for (uint8_t i = 0; i < 16; ++i) {
    pinMode(kAnalogPins[i], INPUT);
  }
  Serial.println(F("Mega analog scanner ready"));
}

void loop() {
  for (uint8_t i = 0; i < 16; ++i) {
    // Discard the first conversion after changing ADC channels.
    analogRead(kAnalogPins[i]);
    const int value = analogRead(kAnalogPins[i]);
    Serial.print('A');
    Serial.print(i);
    Serial.print('=');
    Serial.print(value);
    if (i != 15) Serial.print(' ');
  }
  Serial.println();
  delay(250);
}
