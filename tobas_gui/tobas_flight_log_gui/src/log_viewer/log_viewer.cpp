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
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto grid = new QGridLayout();
  rows->addLayout(grid);

  for (const auto& [idx, plot_tab] : std::views::enumerate(plot_tabs_)) {
    plot_tab = new PlotTabWidget(
      odom_.data,
      setpoint_.data,
      imu_raw_.data,
      imu_filt_.data,
      mag_.data,
      pressure_.data,
      gnss_.data,
      rcin_.data,
      battery_.data,
      cpu_.data,
      cur_rotor_states_.data,
      tar_rotor_speeds_.data,
      cur_joint_states_.data,
      tar_joint_positions_.data,
      tar_joint_velocities_.data,
      tar_joint_efforts_.data,
      ice_cmd_.data,
      pwm_.data,
      sampling_time_.data,
      ctrl_latency_.data,
      vibe_.data,
      repulsive_accel_.data,
      dist_force_.data,
      obsv_fb_.data,
      mr_ctrl_fb_.data);

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

  // キャッシュクリアを忘れるとログを切り替えても以前のものが表示されてしまうことに注意
  odom_.decoder.clearCache();
  setpoint_.decoder.clearCache();
  imu_raw_.decoder.clearCache();
  imu_filt_.decoder.clearCache();
  mag_.decoder.clearCache();
  pressure_.decoder.clearCache();
  gnss_.decoder.clearCache();
  rcin_.decoder.clearCache();
  battery_.decoder.clearCache();
  cpu_.decoder.clearCache();
  cur_rotor_states_.decoder.clearCache();
  tar_rotor_speeds_.decoder.clearCache();
  cur_joint_states_.decoder.clearCache();
  tar_joint_positions_.decoder.clearCache();
  tar_joint_velocities_.decoder.clearCache();
  tar_joint_efforts_.decoder.clearCache();
  ice_cmd_.decoder.clearCache();
  pwm_.decoder.clearCache();
  sampling_time_.decoder.clearCache();
  ctrl_latency_.decoder.clearCache();
  vibe_.decoder.clearCache();
  repulsive_accel_.decoder.clearCache();
  dist_force_.decoder.clearCache();
  obsv_fb_.decoder.clearCache();
  mr_ctrl_fb_.decoder.clearCache();

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
  odom_.data.clear();
  setpoint_.data.clear();
  imu_raw_.data.clear();
  imu_filt_.data.clear();
  mag_.data.clear();
  pressure_.data.clear();
  gnss_.data.clear();
  rcin_.data.clear();
  battery_.data.clear();
  cpu_.data.clear();
  cur_rotor_states_.data.clear();
  tar_rotor_speeds_.data.clear();
  cur_joint_states_.data.clear();
  tar_joint_positions_.data.clear();
  tar_joint_velocities_.data.clear();
  tar_joint_efforts_.data.clear();
  ice_cmd_.data.clear();
  pwm_.data.clear();
  sampling_time_.data.clear();
  ctrl_latency_.data.clear();
  vibe_.data.clear();
  repulsive_accel_.data.clear();
  dist_force_.data.clear();
  obsv_fb_.data.clear();
  mr_ctrl_fb_.data.clear();

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
    try {
      if (topic.ends_with(str::concat('/', topic::kOdometry).data())) {
        odom_.data.append(odom_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kTrajSetpoint).data())) {
        setpoint_.data.append(setpoint_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kImuRaw).data())) {
        imu_raw_.data.append(imu_raw_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kImuFilt).data())) {
        imu_filt_.data.append(imu_filt_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kMagneticField).data())) {
        mag_.data.append(mag_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kAirPressure).data())) {
        pressure_.data.append(pressure_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kGnss).data())) {
        gnss_.data.append(gnss_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kRcInput).data())) {
        rcin_.data.append(rcin_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kBattery).data())) {
        battery_.data.append(battery_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kCpu).data())) {
        cpu_.data.append(cpu_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kRotorStates).data())) {
        cur_rotor_states_.data.append(cur_rotor_states_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kRotorSpeedsCmd).data())) {
        tar_rotor_speeds_.data.append(tar_rotor_speeds_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kJointStates).data())) {
        cur_joint_states_.data.append(cur_joint_states_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kJointPosCmd).data())) {
        tar_joint_positions_.data.append(tar_joint_positions_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kJointVelCmd).data())) {
        tar_joint_velocities_.data.append(tar_joint_velocities_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kJointEffCmd).data())) {
        tar_joint_efforts_.data.append(tar_joint_efforts_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kIcePropulsionSystemCmd).data())) {
        ice_cmd_.data.append(ice_cmd_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kPwmCmd).data())) {
        pwm_.data.append(pwm_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kImuSamplingTime).data())) {
        sampling_time_.data.append(sampling_time_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kControlLatency).data())) {
        ctrl_latency_.data.append(ctrl_latency_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kVibrationLevel).data())) {
        vibe_.data.append(vibe_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kRepulsiveAccel).data())) {
        repulsive_accel_.data.append(repulsive_accel_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kDisturbanceForce).data())) {
        dist_force_.data.append(dist_force_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kObsvFeedback).data())) {
        obsv_fb_.data.append(obsv_fb_.decoder.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(str::concat('/', topic::kMRCtrlFeedback).data())) {
        mr_ctrl_fb_.data.append(mr_ctrl_fb_.decoder.decode(cur_time, ser_data));
      }
    }
    catch (const std::exception& e) {
      qt::qErrorBox(this, "Failed to deserialize \"" + QString::fromStdString(topic) + "\".");
      decode_fail_topics_.insert(topic);
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
