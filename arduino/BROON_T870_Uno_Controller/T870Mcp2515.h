#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "T870CanProtocol.h"

namespace T870Can {

// Minimal MCP2515 driver for an 8 MHz module at 500 kbit/s. Transmit is
// intentionally non-blocking: if all three hardware TX buffers are occupied,
// trySend() returns false immediately instead of delaying the vehicle loop.
class Mcp2515 {
 public:
  explicit Mcp2515(uint8_t chipSelectPin)
      : chipSelectPin_(chipSelectPin), nextTxBuffer_(0U) {}

  bool begin500K8MHz() {
    pinMode(chipSelectPin_, OUTPUT);
    digitalWrite(chipSelectPin_, HIGH);
    SPI.begin();
    reset();
    delay(10);

    if (!setMode(kModeConfiguration)) return false;

    // 8 MHz oscillator, 500 kbit/s. These values match the MCP_CAN_lib
    // timing table: CNF1=0x00, CNF2=0xD1, CNF3=0x81.
    writeRegister(kRegisterCnf1, 0x00U);
    writeRegister(kRegisterCnf2, 0xD1U);
    writeRegister(kRegisterCnf3, 0x81U);

    // Polling-only logger/transmitter. RXB0 rolls over into RXB1 and both
    // buffers accept any valid standard or extended frame.
    writeRegister(kRegisterCanInte, 0x00U);
    writeRegister(kRegisterCanIntf, 0x00U);
    writeRegister(kRegisterRxb0Ctrl, 0x64U);
    writeRegister(kRegisterRxb1Ctrl, 0x60U);

    if (readRegister(kRegisterCnf1) != 0x00U ||
        readRegister(kRegisterCnf2) != 0xD1U ||
        readRegister(kRegisterCnf3) != 0x81U) {
      return false;
    }

    nextTxBuffer_ = 0U;
    return setMode(kModeNormal);
  }

  bool trySend(const Frame& frame) {
    if (frame.id > 0x7FFU || frame.length > kFrameLength) return false;
    if ((errorFlags() & kErrorTransmitBusOff) != 0U) return false;

    for (uint8_t attempt = 0U; attempt < kTxBufferCount; ++attempt) {
      const uint8_t index =
          static_cast<uint8_t>((nextTxBuffer_ + attempt) % kTxBufferCount);
      const uint8_t control = txControlRegister(index);
      if ((readRegister(control) & kTxRequestBit) != 0U) continue;

      uint8_t payload[13];
      payload[0] = static_cast<uint8_t>(frame.id >> 3);
      payload[1] = static_cast<uint8_t>((frame.id & 0x07U) << 5);
      payload[2] = 0U;
      payload[3] = 0U;
      payload[4] = static_cast<uint8_t>(frame.length & 0x0FU);
      for (uint8_t byteIndex = 0U; byteIndex < frame.length; ++byteIndex) {
        payload[5U + byteIndex] = frame.data[byteIndex];
      }

      // Clear stale TX error/arbitration flags, load the selected buffer, and
      // request transmission. There is deliberately no completion wait.
      writeRegister(control, 0x00U);
      writeRegisters(static_cast<uint8_t>(control + 1U), payload,
                     static_cast<uint8_t>(5U + frame.length));
      requestToSend(index);
      nextTxBuffer_ = static_cast<uint8_t>((index + 1U) % kTxBufferCount);
      return true;
    }
    return false;
  }

  bool tryReceive(Frame& frame) {
    const uint8_t interruptFlags = readRegister(kRegisterCanIntf);
    uint8_t receiveFlag = 0U;
    uint8_t standardIdHighRegister = 0U;
    if ((interruptFlags & kReceiveBuffer0Flag) != 0U) {
      receiveFlag = kReceiveBuffer0Flag;
      standardIdHighRegister = kRegisterRxb0Sidh;
    } else if ((interruptFlags & kReceiveBuffer1Flag) != 0U) {
      receiveFlag = kReceiveBuffer1Flag;
      standardIdHighRegister = kRegisterRxb1Sidh;
    } else {
      return false;
    }

    uint8_t header[5];
    readRegisters(standardIdHighRegister, header, sizeof(header));
    const bool extendedFrame = (header[1] & kExtendedIdEnableBit) != 0U;
    uint8_t length = static_cast<uint8_t>(header[4] & 0x0FU);
    if (length > kFrameLength) length = kFrameLength;

    frame.id = static_cast<uint16_t>(
        (static_cast<uint16_t>(header[0]) << 3) |
        (static_cast<uint16_t>(header[1]) >> 5));
    frame.length = length;
    for (uint8_t index = 0U; index < kFrameLength; ++index) {
      frame.data[index] = 0U;
    }
    if (length > 0U) {
      readRegisters(static_cast<uint8_t>(standardIdHighRegister + 5U),
                    frame.data, length);
    }
    bitModify(kRegisterCanIntf, receiveFlag, 0x00U);
    return !extendedFrame;
  }

  uint8_t errorFlags() { return readRegister(kRegisterEflg); }
  uint8_t transmitErrorCount() { return readRegister(kRegisterTec); }
  uint8_t receiveErrorCount() { return readRegister(kRegisterRec); }

 private:
  static constexpr uint8_t kCommandReset = 0xC0U;
  static constexpr uint8_t kCommandRead = 0x03U;
  static constexpr uint8_t kCommandWrite = 0x02U;
  static constexpr uint8_t kCommandBitModify = 0x05U;
  static constexpr uint8_t kCommandRequestToSend = 0x80U;

  static constexpr uint8_t kRegisterTec = 0x1CU;
  static constexpr uint8_t kRegisterRec = 0x1DU;
  static constexpr uint8_t kRegisterCnf3 = 0x28U;
  static constexpr uint8_t kRegisterCnf2 = 0x29U;
  static constexpr uint8_t kRegisterCnf1 = 0x2AU;
  static constexpr uint8_t kRegisterCanInte = 0x2BU;
  static constexpr uint8_t kRegisterCanIntf = 0x2CU;
  static constexpr uint8_t kRegisterEflg = 0x2DU;
  static constexpr uint8_t kRegisterCanStat = 0x0EU;
  static constexpr uint8_t kRegisterCanCtrl = 0x0FU;
  static constexpr uint8_t kRegisterRxb0Ctrl = 0x60U;
  static constexpr uint8_t kRegisterRxb0Sidh = 0x61U;
  static constexpr uint8_t kRegisterRxb1Ctrl = 0x70U;
  static constexpr uint8_t kRegisterRxb1Sidh = 0x71U;

  static constexpr uint8_t kModeMask = 0xE0U;
  static constexpr uint8_t kModeNormal = 0x00U;
  static constexpr uint8_t kModeConfiguration = 0x80U;
  static constexpr uint8_t kTxRequestBit = 0x08U;
  static constexpr uint8_t kErrorTransmitBusOff = 0x20U;
  static constexpr uint8_t kReceiveBuffer0Flag = 0x01U;
  static constexpr uint8_t kReceiveBuffer1Flag = 0x02U;
  static constexpr uint8_t kExtendedIdEnableBit = 0x08U;
  static constexpr uint8_t kTxBufferCount = 3U;

  uint8_t chipSelectPin_;
  uint8_t nextTxBuffer_;

  static uint8_t txControlRegister(uint8_t bufferIndex) {
    if (bufferIndex == 0U) return 0x30U;
    if (bufferIndex == 1U) return 0x40U;
    return 0x50U;
  }

  void select() { digitalWrite(chipSelectPin_, LOW); }
  void deselect() { digitalWrite(chipSelectPin_, HIGH); }

  void beginTransaction() {
    SPI.beginTransaction(SPISettings(4000000UL, MSBFIRST, SPI_MODE0));
    select();
  }

  void endTransaction() {
    deselect();
    SPI.endTransaction();
  }

  void reset() {
    beginTransaction();
    SPI.transfer(kCommandReset);
    endTransaction();
  }

  uint8_t readRegister(uint8_t address) {
    beginTransaction();
    SPI.transfer(kCommandRead);
    SPI.transfer(address);
    const uint8_t value = SPI.transfer(0x00U);
    endTransaction();
    return value;
  }

  void readRegisters(uint8_t address, uint8_t* destination, uint8_t length) {
    beginTransaction();
    SPI.transfer(kCommandRead);
    SPI.transfer(address);
    for (uint8_t index = 0U; index < length; ++index) {
      destination[index] = SPI.transfer(0x00U);
    }
    endTransaction();
  }

  void writeRegister(uint8_t address, uint8_t value) {
    beginTransaction();
    SPI.transfer(kCommandWrite);
    SPI.transfer(address);
    SPI.transfer(value);
    endTransaction();
  }

  void writeRegisters(uint8_t address, const uint8_t* source,
                      uint8_t length) {
    beginTransaction();
    SPI.transfer(kCommandWrite);
    SPI.transfer(address);
    for (uint8_t index = 0U; index < length; ++index) {
      SPI.transfer(source[index]);
    }
    endTransaction();
  }

  void bitModify(uint8_t address, uint8_t mask, uint8_t value) {
    beginTransaction();
    SPI.transfer(kCommandBitModify);
    SPI.transfer(address);
    SPI.transfer(mask);
    SPI.transfer(value);
    endTransaction();
  }

  void requestToSend(uint8_t bufferIndex) {
    beginTransaction();
    SPI.transfer(static_cast<uint8_t>(kCommandRequestToSend |
                                      (1U << bufferIndex)));
    endTransaction();
  }

  bool setMode(uint8_t mode) {
    bitModify(kRegisterCanCtrl, kModeMask, mode);
    const uint32_t startedAtMs = millis();
    while ((readRegister(kRegisterCanStat) & kModeMask) != mode) {
      if (static_cast<uint32_t>(millis() - startedAtMs) >= 20UL) return false;
      delay(1);
    }
    return true;
  }
};

}  // namespace T870Can
