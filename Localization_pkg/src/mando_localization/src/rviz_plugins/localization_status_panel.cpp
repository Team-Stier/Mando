#include "localization_status_panel.hpp"

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>

#include <QBrush>
#include <QColor>
#include <QHeaderView>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <utility>

namespace mando_localization {

LocalizationStatusPanel::LocalizationStatusPanel(QWidget* parent) : rviz::Panel(parent) {
  auto* layout = new QVBoxLayout();
  state_label_ = new QLabel(QStringLiteral("전체 상태: 초기화 중"));
  valid_label_ = new QLabel(QStringLiteral("최종 위치: 사용 불가"));
  status_tree_ = new QTreeWidget();
  status_tree_->setColumnCount(3);
  status_tree_->setHeaderLabels(
      {QStringLiteral("구성요소"), QStringLiteral("상태"), QStringLiteral("설명")});
  status_tree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  status_tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  status_tree_->header()->setSectionResizeMode(2, QHeaderView::Stretch);
  layout->addWidget(state_label_);
  layout->addWidget(valid_label_);
  layout->addWidget(status_tree_);
  setLayout(layout);

  std::string diagnostics_topic;
  std::string state_topic;
  std::string valid_topic;
  double refresh_rate_hz = 0.0;
  configured_ = ros::param::get("/mando_localization/interfaces/topics/status",
                               diagnostics_topic) &&
                ros::param::get("/mando_localization/interfaces/topics/state", state_topic) &&
                ros::param::get("/mando_localization/interfaces/topics/valid", valid_topic) &&
                ros::param::get("/mando_localization/visualization/panel/refresh_rate_hz",
                                refresh_rate_hz) &&
                ros::param::get("/mando_localization/visualization/panel/stale_display_sec",
                                stale_display_sec_);
  const std::string states[] = {"INITIALIZING", "TRACKING", "DEGRADED",
                                "DEAD_RECKONING", "RELOCALIZING", "FAULT"};
  for (const std::string& state : states) {
    std::string label;
    const bool found = ros::param::get(
        "/mando_localization/visualization/panel/korean_labels/" + state,
        label);
    configured_ = configured_ && found && !label.empty();
    korean_state_labels_[state] = label;
  }
  configured_ = configured_ && std::isfinite(refresh_rate_hz) &&
                refresh_rate_hz > 0.0 && std::isfinite(stale_display_sec_) &&
                stale_display_sec_ > 0.0;
  if (configured_) {
    diagnostics_subscriber_ = node_.subscribe(
        diagnostics_topic, 10, &LocalizationStatusPanel::diagnosticsCallback, this);
    state_subscriber_ =
        node_.subscribe(state_topic, 10, &LocalizationStatusPanel::stateCallback, this);
    valid_subscriber_ =
        node_.subscribe(valid_topic, 10, &LocalizationStatusPanel::validCallback, this);
  } else {
    state_label_->setText(QStringLiteral("전체 상태: 인터페이스 YAML 미로딩"));
    state_label_->setStyleSheet(QStringLiteral("QLabel { color: #d32f2f; font-weight: bold; }"));
  }

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &LocalizationStatusPanel::refreshDisplay);
  refresh_timer_->start(configured_
                            ? static_cast<int>(1000.0 / refresh_rate_hz)
                            : 200);
}

void LocalizationStatusPanel::diagnosticsCallback(
    const diagnostic_msgs::DiagnosticArrayConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  diagnostics_ = *message;
  have_diagnostics_ = true;
  last_diagnostics_wall_time_ = ros::WallTime::now();
}

void LocalizationStatusPanel::stateCallback(const std_msgs::StringConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = message->data;
}

void LocalizationStatusPanel::validCallback(const std_msgs::BoolConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  valid_ = message->data;
}

void LocalizationStatusPanel::refreshDisplay() {
  if (!configured_) {
    return;
  }
  diagnostic_msgs::DiagnosticArray diagnostics;
  std::string state;
  bool valid = false;
  bool have_diagnostics = false;
  ros::WallTime last_diagnostics_wall_time;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    diagnostics = diagnostics_;
    state = state_;
    valid = valid_;
    have_diagnostics = have_diagnostics_;
    last_diagnostics_wall_time = last_diagnostics_wall_time_;
  }

  state_label_->setText(QStringLiteral("전체 상태: ") + koreanState(state) +
                        QStringLiteral(" (") + QString::fromStdString(state) +
                        QStringLiteral(")"));
  valid_label_->setText(valid ? QStringLiteral("최종 위치: 사용 가능")
                              : QStringLiteral("최종 위치: 사용 불가"));
  valid_label_->setStyleSheet(
      valid ? QStringLiteral("QLabel { color: #2e7d32; font-weight: bold; }")
            : QStringLiteral("QLabel { color: #d32f2f; font-weight: bold; }"));

  status_tree_->clear();
  if (!have_diagnostics) {
    auto* item = new QTreeWidgetItem(status_tree_);
    item->setText(0, QStringLiteral("진단"));
    item->setText(1, QStringLiteral("대기"));
    item->setText(2, QStringLiteral("DiagnosticArray 수신 대기 중"));
    return;
  }
  if (last_diagnostics_wall_time.isZero() ||
      (ros::WallTime::now() - last_diagnostics_wall_time).toSec() >
          stale_display_sec_) {
    auto* item = new QTreeWidgetItem(status_tree_);
    item->setText(0, QStringLiteral("진단"));
    item->setText(1, QStringLiteral("갱신 지연"));
    item->setText(2, QStringLiteral("마지막 DiagnosticArray가 오래되었습니다"));
    item->setForeground(1, QBrush(QColor("#d32f2f")));
  }
  for (const diagnostic_msgs::DiagnosticStatus& status : diagnostics.status) {
    auto* item = new QTreeWidgetItem(status_tree_);
    item->setText(0, koreanComponent(status.name));
    item->setText(1, koreanLevel(status.level));
    item->setText(2, QString::fromStdString(status.message));
    QColor color("#2e7d32");
    if (status.level == diagnostic_msgs::DiagnosticStatus::WARN) {
      color = QColor("#ed6c02");
    } else if (status.level >= diagnostic_msgs::DiagnosticStatus::ERROR) {
      color = QColor("#d32f2f");
    }
    item->setForeground(1, QBrush(color));
  }
}

QString LocalizationStatusPanel::koreanState(const std::string& state) const {
  const auto found = korean_state_labels_.find(state);
  if (found == korean_state_labels_.end()) {
    return QStringLiteral("알 수 없음");
  }
  return QString::fromStdString(found->second);
}

QString LocalizationStatusPanel::koreanLevel(const uint8_t level) {
  if (level == diagnostic_msgs::DiagnosticStatus::OK) return QStringLiteral("정상");
  if (level == diagnostic_msgs::DiagnosticStatus::WARN) return QStringLiteral("경고");
  if (level == diagnostic_msgs::DiagnosticStatus::ERROR) return QStringLiteral("오류");
  return QStringLiteral("정지");
}

QString LocalizationStatusPanel::koreanComponent(const std::string& component) {
  if (component == "LOCALIZATION") return QStringLiteral("전체 Localization");
  if (component == "IMU") return QStringLiteral("IMU 토픽");
  if (component == "ENCODER") return QStringLiteral("엔코더 토픽");
  if (component == "ENCODER_TWIST") return QStringLiteral("차량 Twist");
  if (component == "LOCAL_ODOMETRY") return QStringLiteral("Local EKF");
  if (component == "GPS_POSE") return QStringLiteral("GPS 보정 위치");
  if (component == "LIDAR_POSE") return QStringLiteral("LiDAR 보정 위치");
  if (component == "GLOBAL_ODOMETRY") return QStringLiteral("Global EKF");
  if (component == "IMU_DEVICE") return QStringLiteral("IMU USB 장치");
  if (component == "GPS_DEVICE") return QStringLiteral("GPS USB 장치");
  if (component == "IMU_DRIVER") return QStringLiteral("IMU 드라이버");
  if (component == "GPS_DRIVER") return QStringLiteral("GPS 드라이버");
  return QString::fromStdString(component);
}

}  // namespace mando_localization

PLUGINLIB_EXPORT_CLASS(mando_localization::LocalizationStatusPanel, rviz::Panel)
