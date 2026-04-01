// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/log_viewer.hpp"

#include <ranges>

#include <QDebug>
#include <QGridLayout>
#include <QVBoxLayout>

#include <tobas_constants/path.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ros2_tools/rosbag.hpp>
#include <tobas_ros2_tools/util.hpp>

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
      odom_data_,
      setpoint_data_,
      raw_imu_data_,
      filt_imu_data_,
      mag_data_,
      gnss_data_,
      rcin_data_,
      battery_data_,
      cpu_data_,
      cur_rotor_states_data_,
      tar_rotor_speeds_data_,
      cur_joint_states_data_,
      tar_joint_positions_data_,
      tar_joint_velocities_data_,
      tar_joint_efforts_data_,
      ice_cmd_data_,
      sampling_time_data_,
      ctrl_latency_data_,
      vibe_data_,
      dist_force_data_,
      obsv_fb_data_,
      mr_ctrl_fb_data_);

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

  odom_cov_decoder_.clearCache();
  mag_decoder_.clearCache();
  gnss_decoder_.clearCache();
  rcin_decoder_.clearCache();
  battery_decoder_.clearCache();
  cpu_decoder_.clearCache();
  rotor_states_decoder_.clearCache();
  rotor_speeds_decoder_.clearCache();
  joint_states_decoder_.clearCache();
  joint_commands_decoder_.clearCache();
  ice_cmd_decoder_.clearCache();
  latency_decoder_.clearCache();
  vibe_decoder_.clearCache();
  wrench_decoder_.clearCache();
  obsv_fb_decoder_.clearCache();
  mr_ctrl_fb_decoder_.clearCache();

  for (auto& plot_tab : plot_tabs_) {
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
  odom_data_.clear();
  setpoint_data_.clear();
  raw_imu_data_.clear();
  filt_imu_data_.clear();
  mag_data_.clear();
  gnss_data_.clear();
  rcin_data_.clear();
  battery_data_.clear();
  cpu_data_.clear();
  cur_rotor_states_data_.clear();
  tar_rotor_speeds_data_.clear();
  cur_joint_states_data_.clear();
  tar_joint_positions_data_.clear();
  tar_joint_velocities_data_.clear();
  tar_joint_efforts_data_.clear();
  ice_cmd_data_.clear();
  sampling_time_data_.clear();
  ctrl_latency_data_.clear();
  vibe_data_.clear();
  dist_force_data_.clear();
  obsv_fb_data_.clear();
  mr_ctrl_fb_data_.clear();

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
      if (topic.ends_with(path::join('/', topic::kOdometry))) {
        odom_data_.push_back(odom_cov_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kTrajSetpoint))) {
        setpoint_data_.push_back(odom_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kImuRaw))) {
        raw_imu_data_.push_back(imu_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kImuFilt))) {
        filt_imu_data_.push_back(imu_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kMagneticField))) {
        mag_data_.push_back(mag_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kGnss))) {
        gnss_data_.push_back(gnss_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kRcInput))) {
        rcin_data_.push_back(rcin_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kBattery))) {
        battery_data_.push_back(battery_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kCpu))) {
        cpu_data_.push_back(cpu_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kRotorStates))) {
        cur_rotor_states_data_.push_back(rotor_states_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kRotorSpeedsCmd))) {
        tar_rotor_speeds_data_.push_back(rotor_speeds_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kJointStates))) {
        cur_joint_states_data_.push_back(joint_states_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kJointPosCmd))) {
        tar_joint_positions_data_.push_back(joint_commands_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kJointVelCmd))) {
        tar_joint_velocities_data_.push_back(joint_commands_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kJointEffCmd))) {
        tar_joint_efforts_data_.push_back(joint_commands_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kIcePropulsionSystemCmd))) {
        ice_cmd_data_.push_back(ice_cmd_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kImuSamplingTime))) {
        sampling_time_data_.push_back(latency_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kControlLatency))) {
        ctrl_latency_data_.push_back(latency_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kVibrationLevel))) {
        vibe_data_.push_back(vibe_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kDisturbanceForce))) {
        dist_force_data_.push_back(wrench_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kObsvFeedback))) {
        obsv_fb_data_.push_back(obsv_fb_decoder_.decode(cur_time, ser_data));
      }
      else if (topic.ends_with(path::join('/', topic::kMRCtrlFeedback))) {
        mr_ctrl_fb_data_.push_back(mr_ctrl_fb_decoder_.decode(cur_time, ser_data));
      }
    }
    catch (const std::exception& e) {
      qt::qErrorBox(this, "Failed to deserialize \"" + QString::fromStdString(topic) + "\".");
      decode_fail_topics_.insert(topic);
    }
  }

  // データをプロット
  for (auto& plot_tab : plot_tabs_) {
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
