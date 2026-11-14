// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/log_viewer.hpp"

#include <ranges>

#include <QDebug>
#include <QGridLayout>
#include <QVBoxLayout>

#include <tobas_constants/path.hpp>
#include <tobas_constants/ros.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ros2_tools/rosbag.hpp>
#include <tobas_ros2_tools/util.hpp>

namespace fs = std::filesystem;

namespace tobas::gui::log
{
namespace
{
constexpr int kPlotRows = 3;
constexpr int kPlotColumns = 2;

constexpr double kWindowDuration = 5.0;  // [s]
}  // namespace

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
  const auto additional_accel = addData<tobas_kdl_msgs::msg::AccelStamped>(topic::kAdditionalAccel);
  const auto dist_force = addData<tobas_kdl_msgs::msg::WrenchStamped>(topic::kDisturbanceForce);
  const auto obsv_fb = addData<tobas_debug_msgs::msg::ObserverFeedback>(topic::kObsvFeedback);
  const auto mr_ctrl_fb = addData<tobas_debug_msgs::msg::MulticopterControllerFeedback>(topic::kMRCtrlFeedback);

  const auto rows = new QVBoxLayout(this);

  plot_grid_ = new QGridLayout();
  plot_grid_->setContentsMargins(0, 0, 0, 0);
  plot_grid_->setSpacing(4);
  rows->addLayout(plot_grid_);

  // Place plot panels in even grid rows and columns, reserving the gaps for separators.
  for (const auto& [idx, plot_panel] : std::views::enumerate(plot_panels_)) {
    plot_panel = new PlotPanelWidget(
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
      additional_accel->getValues(),
      dist_force->getValues(),
      obsv_fb->getValues(),
      mr_ctrl_fb->getValues());

    plot_panel->setCurrentIndex(idx);

    const auto row = 2 * (idx % kPlotRows);
    const auto col = 2 * (idx / kPlotRows);
    plot_grid_->addWidget(plot_panel, row, col, 1, 1);
    plot_grid_->setRowStretch(row, 1);
    plot_grid_->setColumnStretch(col, 1);

    connect(
      plot_panel,
      &PlotPanelWidget::expandedChanged,
      this,
      [this, plot_panel](bool _expanded) { setExpandedPanel(_expanded ? plot_panel : nullptr); });
  }

  // Add one vertical and two horizontal separators between the plot panels.
  for (const auto& [idx, separator] : std::views::enumerate(plot_separators_)) {
    separator = new QFrame();
    separator->setFrameShadow(QFrame::Sunken);
    if (idx == 0) {
      separator->setFrameShape(QFrame::VLine);
      plot_grid_->addWidget(separator, 0, 1, 2 * kPlotRows - 1, 1);
    }
    else {
      separator->setFrameShape(QFrame::HLine);
      plot_grid_->addWidget(separator, 2 * idx - 1, 0, 1, 2 * kPlotColumns - 1);
    }
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

  for (const auto& plot_panel : plot_panels_) {
    plot_panel->clear();
    plot_panel->setTimeScale(0.0, kWindowDuration);
  }

  playback_ctrl_->reset();
  playback_ctrl_->setEnabled(false);
}

void FlightLogViewerWidget::setLogName(const QString& _log_name)
{
  reset();

  // Update the absolute rosbag path.
  log_path_ = ros2::expandUser(kRosbagDirHome) / _log_name.toStdString();

  // Open the rosbag.
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

  // Update the log length.
  const auto& metadata = reader_.get_metadata();
  const auto duration = metadata.duration.count() * 1e-9;  // [s]
  playback_ctrl_->setDuration(std::max(duration - kWindowDuration, 0.0));
  playback_ctrl_->setEnabled(true);

  // Show the log at time 0.
  setPlotData(0.0);
}

bool FlightLogViewerWidget::open(const std::string& _rosbag_path)
{
  try {
    reader_.open(_rosbag_path);
  }
  catch (const std::exception& e) {
    qWarning() << "Failed to open" << QString::fromStdString(_rosbag_path) + ":" << e.what();
    return false;
  }

  return true;
}

void FlightLogViewerWidget::setPlotData(double _time_from_start)
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
  const auto record_start_time = metadata.starting_time.time_since_epoch().count();               // [ns]
  const auto window_start_time = record_start_time + static_cast<long>(_time_from_start * 1e+9);  // [ns]
  const auto window_stop_time = window_start_time + static_cast<long>(kWindowDuration * 1e+9);    // [ns]

  // Move to the initial time.
  reader_.seek(window_start_time);

  // Initialize data.
  for (const auto& elem : data_) {
    elem->clearValues();
  }

  // Classify data.
  while (reader_.has_next()) {
    const auto bag_msg = reader_.read_next();

    const auto& ser_data = bag_msg->serialized_data;
    const auto& cur_time = bag_msg->recv_timestamp;  // [ns]
    const auto& topic = bag_msg->topic_name;

    if (cur_time > window_stop_time) {
      break;
    }

    // Do not decode topics that failed once until the log is reset.
    if (decode_fail_topics_.contains(topic)) {
      continue;
    }

    // Decode.
    for (const auto& elem : data_) {
      if (topic.ends_with(elem->getTopic())) {
        if (!elem->decode(cur_time, ser_data)) {
          qt::qErrorBox(this, "Failed to deserialize '" + QString::fromStdString(topic) + "'.");
          decode_fail_topics_.insert(topic);
        }
        break;
      }
    }
  }

  // Plot data.
  for (const auto& plot_panel : plot_panels_) {
    // The plot becomes slightly distorted unless the range is set before setting data.
    plot_panel->setTimeScale(window_start_time * 1e-9, window_stop_time * 1e-9);
    plot_panel->plot();
  }
}

void FlightLogViewerWidget::setExpandedPanel(PlotPanelWidget* _expanded_panel)
{
  for (const auto& [idx, plot_panel] : std::views::enumerate(plot_panels_)) {
    plot_grid_->removeWidget(plot_panel);
    if (plot_panel == _expanded_panel) {
      plot_grid_->addWidget(plot_panel, 0, 0, 2 * kPlotRows - 1, 2 * kPlotColumns - 1);
    }
    else {
      plot_grid_->addWidget(plot_panel, 2 * (idx % kPlotRows), 2 * (idx / kPlotRows), 1, 1);
    }
    plot_panel->setVisible(!_expanded_panel || plot_panel == _expanded_panel);
  }

  for (const auto& separator : plot_separators_) {
    separator->setVisible(!_expanded_panel);
  }
}

void FlightLogViewerWidget::onPlaybackTimeChanged(double _time_from_start)
{
  setPlotData(_time_from_start);
}
}  // namespace tobas::gui::log
