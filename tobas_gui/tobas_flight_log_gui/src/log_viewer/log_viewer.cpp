// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/log_viewer.hpp"

#include <ranges>

#include <QDebug>
#include <QGridLayout>
#include <QVBoxLayout>

#include <tobas_constants/path.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ros2_tools/rosbag.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_string_tools/chars.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace log
{
FlightLogViewerWidget::FlightLogViewerWidget()
{
  const auto odom = addData<tobas_msgs::msg::OdometryWithCovarianceStamped>(topic::kOdometry);
  const auto setpoint = addData<tobas_msgs::msg::OdometryStamped>(topic::kTrajSetpoint);
  const auto imu_raw = addData<tobas_msgs::msg::Imu>(topic::kImuRaw);
  const auto imu_filt = addData<tobas_msgs::msg::Imu>(topic::kImuFilt);
  const auto mag = addData<tobas_msgs::msg::MagneticField>(topic::kMagneticField);
  const auto pressure = addData<tobas_msgs::msg::FluidPressure>(topic::kAirPressure);
  const auto gnss = addData<tobas_msgs::msg::Gnss>(topic::kGnss);
  const auto rcin = addData<tobas_msgs::msg::RCInput>(topic::kRcInput);
  const auto battery = addData<tobas_msgs::msg::Battery>(topic::kBattery);
  const auto cpu = addData<tobas_msgs::msg::Cpu>(topic::kCpu);
  const auto cur_rotor_states = addData<tobas_msgs::msg::RotorStateArray>(topic::kRotorStates);
  const auto tar_rotor_speeds = addData<tobas_msgs::msg::RotorSpeedArray>(topic::kRotorSpeedsCmd);
  const auto cur_joint_states = addData<tobas_msgs::msg::JointStateArray>(topic::kJointStates);
  const auto tar_joint_positions = addData<tobas_msgs::msg::JointCommandArray>(topic::kJointPosCmd);
  const auto tar_joint_velocities = addData<tobas_msgs::msg::JointCommandArray>(topic::kJointVelCmd);
  const auto tar_joint_efforts = addData<tobas_msgs::msg::JointCommandArray>(topic::kJointEffCmd);
  const auto ice_cmd = addData<tobas_msgs::msg::IcePropulsionSystemCommand>(topic::kIcePropulsionSystemCmd);
  const auto pwm = addData<tobas_msgs::msg::PwmArray>(topic::kPwmCmd);
  const auto sampling_time = addData<tobas_msgs::msg::Latency>(topic::kImuSamplingTime);
  const auto ctrl_latency = addData<tobas_msgs::msg::Latency>(topic::kControlLatency);
  const auto vibe = addData<tobas_msgs::msg::VibrationLevel>(topic::kVibrationLevel);
  const auto repulsive_accel = addData<tobas_msgs::msg::RepulsiveAcceleration>(topic::kRepulsiveAccel);
  const auto dist_force = addData<tobas_kdl_msgs::msg::WrenchStamped>(topic::kDisturbanceForce);
  const auto obsv_fb = addData<tobas_debug_msgs::msg::ObserverFeedback>(topic::kObsvFeedback);
  const auto mr_ctrl_fb = addData<tobas_debug_msgs::msg::MulticopterControllerFeedback>(topic::kMRCtrlFeedback);

  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto grid = new QGridLayout();
  rows->addLayout(grid);

  for (const auto& [idx, plot_tab] : std::views::enumerate(plot_tabs_)) {
    plot_tab = new PlotTabWidget(
      odom->getValues(),
      setpoint->getValues(),
      imu_raw->getValues(),
      imu_filt->getValues(),
      mag->getValues(),
      pressure->getValues(),
      gnss->getValues(),
      rcin->getValues(),
      battery->getValues(),
      cpu->getValues(),
      cur_rotor_states->getValues(),
      tar_rotor_speeds->getValues(),
      cur_joint_states->getValues(),
      tar_joint_positions->getValues(),
      tar_joint_velocities->getValues(),
      tar_joint_efforts->getValues(),
      ice_cmd->getValues(),
      pwm->getValues(),
      sampling_time->getValues(),
      ctrl_latency->getValues(),
      vibe->getValues(),
      repulsive_accel->getValues(),
      dist_force->getValues(),
      obsv_fb->getValues(),
      mr_ctrl_fb->getValues());

    plot_tab->setCurrentIndex(idx);

    const auto row = idx % 3;
    const auto col = idx / 3;
    grid->addWidget(plot_tab, row, col, 1, 1);
    grid->setRowStretch(row, 1);
    grid->setColumnStretch(col, 1);
  }

  playback_ctrl_ = new PlaybackControlWidget();
  rows->addWidget(playback_ctrl_, 0);
  connect(playback_ctrl_, &PlaybackControlWidget::timeChanged, this, &self::onPlaybackTimeChanged);

  reset();
}

void FlightLogViewerWidget::reset()
{
  log_path_.clear();
  reader_.close();
  decode_fail_topics_.clear();

  for (const auto& elem : data_) {
    elem->clearCache();
  }

  for (const auto& plot_tab : plot_tabs_) {
    plot_tab->clear();
    plot_tab->setTimeScale(0., kWindowDuration);
  }

  playback_ctrl_->reset();
}

void FlightLogViewerWidget::setLogName(const QString& log_name)
{
  reset();

  // rosbagの絶対パスを更新
  log_path_ = ros2::expandUser(kRosbagDirHome) / log_name.toStdString();

  // rosbagを開く
  if (!open(log_path_)) {
    if (!ros2::reindexRosBag(log_path_)) {
      qt::qErrorBox(this, "The log file is broken and failed to fix it.");
      return;
    }
    if (!open(log_path_)) {
      qt::qErrorBox(this, "Failed to open the log file. The data is probably corrupted.");
      return;
    }
  }

  // ログの長さを更新
  const auto& metadata = reader_.get_metadata();
  const auto duration = metadata.duration.count() * 1e-9;  // [s]
  playback_ctrl_->setDuration(std::max(duration - kWindowDuration, 0.));

  // 時刻0のログを表示
  setPlotData(0.);
}

bool FlightLogViewerWidget::open(const std::string& rosbag_path)
{
  try {
    reader_.open(rosbag_path);
  }
  catch (const std::exception& e) {
    qWarning() << "Failed to open" << QString::fromStdString(rosbag_path) + ":" << e.what();
    return false;
  }

  return true;
}

void FlightLogViewerWidget::setPlotData(double time_from_start)
{
  if (log_path_.empty()) {
    qWarning() << "Log path is not set.";
    return;
  }

  if (!fs::exists(log_path_)) {
    qWarning() << "Log path" << QString::fromStdString(log_path_) << "does not exist.";
    return;
  }

  const auto& metadata = reader_.get_metadata();
  const auto record_start_time = metadata.starting_time.time_since_epoch().count();              // [ns]
  const auto window_start_time = record_start_time + static_cast<long>(time_from_start * 1e+9);  // [ns]
  const auto window_stop_time = window_start_time + static_cast<long>(kWindowDuration * 1e+9);   // [ns]

  // 初期時刻に移動
  reader_.seek(window_start_time);

  // データを初期化
  for (const auto& elem : data_) {
    elem->clearValues();
  }

  // データを仕分ける
  while (reader_.has_next()) {
    const auto bag_msg = reader_.read_next();

    const auto& ser_data = bag_msg->serialized_data;
    const auto& cur_time = bag_msg->recv_timestamp;  // [ns]
    const auto& topic = bag_msg->topic_name;

    if (cur_time > window_stop_time) {
      break;
    }

    // 一度デコードに失敗したトピックはログがリセットされるまでデコードしない
    if (decode_fail_topics_.contains(topic)) {
      continue;
    }

    // デコード
    for (const auto& elem : data_) {
      if (topic.ends_with(elem->getTopic())) {
        if (!elem->decode(cur_time, ser_data)) {
          qt::qErrorBox(this, "Failed to deserialize \"" + QString::fromStdString(topic) + "\".");
          decode_fail_topics_.insert(topic);
        }
        break;
      }
    }
  }

  // データをプロット
  for (const auto& plot_tab : plot_tabs_) {
    // データの設定の前に範囲を指定しないと若干プロットが崩れる
    plot_tab->setTimeScale(window_start_time * 1e-9, window_stop_time * 1e-9);
    plot_tab->plot();
  }
}

void FlightLogViewerWidget::onPlaybackTimeChanged(double time_from_start)
{
  setPlotData(time_from_start);
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
