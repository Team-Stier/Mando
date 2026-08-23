#pragma once

#include "BuildOptions.h"

#if BROON_ENABLE_ROS

#include <Arduino.h>
#include <ros.h>
#include <erp42_msgs/DriveCmd.h>
#include <erp42_msgs/SerialFeedBack.h>

#include "BroonT870Core.h"

namespace BroonT870Controller {

// NodeHandle_ serializes TopicInfo into message_out after a seven-byte frame
// prefix and adds a trailing checksum. The two advertised topics require at
// most 97 payload bytes (/erp42_serial/feedback) and therefore 105 frame
// bytes. SerialFeedBack is 26 payload bytes (two AVR float64 encodings) or 34
// frame bytes; DriveCmd is five input bytes. 192-byte buffers leave 87 bytes
// beyond the largest negotiated frame while avoiding the Uno default's 280 B
// input and output buffers. This bridge owns exactly one subscriber and one
// publisher, so allocating rosserial's default 25 slots per direction is not
// justified.
constexpr int kRosBridgeMaxSubscribers = 1;
constexpr int kRosBridgeMaxPublishers = 1;
constexpr int kRosBridgeTransportBufferBytes = 192;
constexpr int kRosBridgeLargestTopicInfoFrameBytes = 105;
constexpr int kRosBridgeSerialFeedbackFrameBytes = 34;

static_assert(kRosBridgeTransportBufferBytes >=
                  kRosBridgeLargestTopicInfoFrameBytes,
              "ROS transport buffer must fit TopicInfo negotiation");
static_assert(kRosBridgeTransportBufferBytes >=
                  kRosBridgeSerialFeedbackFrameBytes,
              "ROS transport buffer must fit SerialFeedBack");

using RosBridgeNodeHandle =
    ros::NodeHandle_<ArduinoHardware, kRosBridgeMaxSubscribers,
                     kRosBridgeMaxPublishers, kRosBridgeTransportBufferBytes,
                     kRosBridgeTransportBufferBytes>;

struct RosDriveReceipt {
  uint16_t kph;
  int16_t degrees;
  uint8_t brake;
  uint32_t receivedAtMs;
  bool received;
};

class RosBridge {
 public:
  RosBridge()
      : driveSubscriber_("/erp42_serial/drive", &RosBridge::onDriveCommand),
        feedbackPublisher_("/erp42_serial/feedback", &feedbackMessage_),
        receipt_{0U, 0, 0U, 0UL, false},
        aliveCounter_(0U) {}

  void begin() {
    activeBridge_ = this;
    node_.initNode();
    node_.subscribe(driveSubscriber_);
    node_.advertise(feedbackPublisher_);
  }

  void spinOnce() {
    node_.spinOnce();
  }

  BroonT870::VehicleCommand vehicleCommand(
      uint32_t nowMs, uint16_t timeoutMs, int16_t maximumAbsKph,
      int16_t minimumDegrees, int16_t maximumDegrees, int16_t leftAdc,
      int16_t centerAdc, int16_t rightAdc) const {
    BroonT870::VehicleCommand command = {0, 0, 0.0f, BroonT870::MODE_ROS,
                                         nowMs, false, false};
    if (!receipt_.received ||
        BroonT870::hasElapsed(nowMs, receipt_.receivedAtMs, timeoutMs)) {
      return command;
    }
    if (maximumAbsKph <= 0) {
      return command;
    }

    // DriveCmd.KPH is uint16_t in the installed ERP42 message definition.
    // Do not reinterpret it as a reverse request without an upstream gear
    // policy. The pure mapper remains signed for future signed interfaces.
    const uint16_t limitedKph =
        receipt_.kph > static_cast<uint16_t>(maximumAbsKph)
            ? static_cast<uint16_t>(maximumAbsKph)
            : receipt_.kph;
    command.targetSpeedKph = static_cast<float>(limitedKph);
    command.steerTargetAdc = BroonT870::degreesToSteerAdc(
        receipt_.degrees, minimumDegrees, maximumDegrees, leftAdc, centerAdc,
        rightAdc);
    command.receivedAtMs = receipt_.receivedAtMs;
    command.valid = true;
    command.brakeRequested = receipt_.brake != 0U;
    return command;
  }

  void publishFeedback(const BroonT870::EncoderMeasurement& frontEncoder,
                        int16_t steeringAdc, BroonT870::ControlMode controlMode,
                        bool remoteStopActive, bool stopRequested) {
    const BroonT870::FeedbackStatus status = BroonT870::nextFeedbackStatus(
        remoteStopActive, stopRequested, aliveCounter_);
    aliveCounter_ = status.alive;
    feedbackMessage_.MorA =
        controlMode == BroonT870::MODE_ROS ? 1U : 0U;
    feedbackMessage_.EStop = status.estop;
    feedbackMessage_.speed = frontEncoder.speedMps;
    feedbackMessage_.steer = static_cast<float>(steeringAdc);
    feedbackMessage_.brake = status.brake;
    feedbackMessage_.encoder = frontEncoder.deltaCount;
    feedbackMessage_.alive = status.alive;
    feedbackPublisher_.publish(&feedbackMessage_);
  }

 private:
  static void onDriveCommand(const erp42_msgs::DriveCmd& message) {
    if (activeBridge_ == 0) {
      return;
    }
    // The callback is receive-only: it does not map commands or touch pins.
    activeBridge_->receipt_.kph = message.KPH;
    activeBridge_->receipt_.degrees = message.Deg;
    activeBridge_->receipt_.brake = message.brake;
    activeBridge_->receipt_.receivedAtMs = millis();
    activeBridge_->receipt_.received = true;
  }

  RosBridgeNodeHandle node_;
  erp42_msgs::SerialFeedBack feedbackMessage_;
  ros::Subscriber<erp42_msgs::DriveCmd> driveSubscriber_;
  ros::Publisher feedbackPublisher_;
  RosDriveReceipt receipt_;
  uint8_t aliveCounter_;

  static RosBridge* activeBridge_;
};

RosBridge* RosBridge::activeBridge_ = 0;

}  // namespace BroonT870Controller

#endif  // BROON_ENABLE_ROS
