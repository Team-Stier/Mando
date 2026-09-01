#pragma once

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <QLabel>
#include <QTimer>
#include <QTreeWidget>

#include <mutex>
#include <map>
#include <string>
#include <vector>

namespace mando_localization {

/**
 * @brief LocalizationStatusManager 결과를 한국어로 표시하는 관측 전용 RViz 패널.
 *
 * 상태를 재판정하거나 제어 토픽을 발행하지 않는다. ROS callback에서 받은
 * 스냅샷은 mutex로 보호하고 Qt GUI thread의 timer에서 화면에 반영한다.
 */
class LocalizationStatusPanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit LocalizationStatusPanel(QWidget* parent = nullptr);

 private Q_SLOTS:
  void refreshDisplay();

 private:
  void diagnosticsCallback(const diagnostic_msgs::DiagnosticArrayConstPtr& message);
  void stateCallback(const std_msgs::StringConstPtr& message);
  void validCallback(const std_msgs::BoolConstPtr& message);
  QString koreanState(const std::string& state) const;
  static QString koreanLevel(uint8_t level);
  static QString koreanComponent(const std::string& component);

  ros::NodeHandle node_;
  ros::Subscriber diagnostics_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber valid_subscriber_;
  QLabel* state_label_{nullptr};
  QLabel* valid_label_{nullptr};
  QTreeWidget* status_tree_{nullptr};
  QTimer* refresh_timer_{nullptr};

  std::mutex mutex_;
  diagnostic_msgs::DiagnosticArray diagnostics_;
  std::string state_{"INITIALIZING"};
  bool valid_{false};
  bool have_diagnostics_{false};
  bool configured_{false};
  double stale_display_sec_{1.0};
  ros::WallTime last_diagnostics_wall_time_;
  std::map<std::string, std::string> korean_state_labels_;
};

}  // namespace mando_localization
