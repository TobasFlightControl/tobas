// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

#include <rosbag2_cpp/reader.hpp>

#include "./message_decoder.hpp"
#include "./playback_control.hpp"
#include "./plot_tab.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class FlightLogViewerWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogViewerWidget;
  using super = QWidget;

  static constexpr double kWindowDuration = 5.;  // [s]

public:
  explicit FlightLogViewerWidget();

  void reset();

  void setLogName(const QString& log_name);

private:
  template <typename MsgType>
  struct DataDecoder
  {
    QVector<MsgType> data;
    MessageDecoderCache<MsgType> decoder;
  };

  DataDecoder<tobas_msgs::msg::OdometryWithCovarianceStamped> odom_;
  DataDecoder<tobas_msgs::msg::OdometryStamped> setpoint_;
  DataDecoder<tobas_msgs::msg::Imu> imu_raw_;
  DataDecoder<tobas_msgs::msg::Imu> imu_filt_;
  DataDecoder<tobas_msgs::msg::MagneticField> mag_;
  DataDecoder<tobas_msgs::msg::FluidPressure> pressure_;
  DataDecoder<tobas_msgs::msg::Gnss> gnss_;
  DataDecoder<tobas_msgs::msg::RCInput> rcin_;
  DataDecoder<tobas_msgs::msg::Battery> battery_;
  DataDecoder<tobas_msgs::msg::Cpu> cpu_;
  DataDecoder<tobas_msgs::msg::RotorStateArray> cur_rotor_states_;
  DataDecoder<tobas_msgs::msg::RotorSpeedArray> tar_rotor_speeds_;
  DataDecoder<tobas_msgs::msg::JointStateArray> cur_joint_states_;
  DataDecoder<tobas_msgs::msg::JointCommandArray> tar_joint_positions_;
  DataDecoder<tobas_msgs::msg::JointCommandArray> tar_joint_velocities_;
  DataDecoder<tobas_msgs::msg::JointCommandArray> tar_joint_efforts_;
  DataDecoder<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_;
  DataDecoder<tobas_msgs::msg::PwmArray> pwm_;
  DataDecoder<tobas_msgs::msg::Latency> sampling_time_;
  DataDecoder<tobas_msgs::msg::Latency> ctrl_latency_;
  DataDecoder<tobas_msgs::msg::VibrationLevel> vibe_;
  DataDecoder<tobas_msgs::msg::RepulsiveAcceleration> repulsive_accel_;
  DataDecoder<tobas_kdl_msgs::msg::WrenchStamped> dist_force_;
  DataDecoder<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb_;
  DataDecoder<tobas_debug_msgs::msg::MulticopterControllerFeedback> mr_ctrl_fb_;

  std::filesystem::path log_path_;
  std::unordered_set<std::string> decode_fail_topics_;
  rosbag2_cpp::Reader reader_;

  std::array<PlotTabWidget*, 6> plot_tabs_;
  PlaybackControlWidget* playback_ctrl_;

  bool open(const std::string& rosbag_path);

  void setPlotData(double time_from_start);

private Q_SLOTS:
  void onPlaybackTimeChanged(double time_from_start);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
