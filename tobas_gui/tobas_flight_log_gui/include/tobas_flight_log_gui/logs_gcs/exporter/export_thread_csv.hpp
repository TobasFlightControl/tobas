// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <fstream>

#include <rosbag2_cpp/reader.hpp>

#include <tobas_debug_msgs/msg/multicopter_controller_feedback.hpp>
#include <tobas_debug_msgs/msg/observer_feedback.hpp>
#include <tobas_kdl_msgs/msg/wrench_stamped.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/gnss.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/imu.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs/msg/magnetic_field.hpp>
#include <tobas_msgs/msg/odometry_stamped.hpp>
#include <tobas_msgs/msg/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/repulsive_acceleration.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/vibration_level.hpp>

#include "../message_decoder.hpp"
#include "./export_thread.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class ExportThreadCsv : public ExportThread
{
  Q_OBJECT

  using Time = rcutils_time_point_value_t;
  using SerializedDataMap = std::map<std::string, std::shared_ptr<rcutils_uint8_array_t>>;
  using HistoryMap = std::map<Time, SerializedDataMap>;

  static constexpr Time kExpirationTime = 1'000'000'000;  // [ns]

public:
  explicit ExportThreadCsv(const QString& log_name, const QString& save_path);

  void run() override;

private:
  const QString log_name_;
  const QString save_path_;

  std::vector<std::string> rotor_link_names_;
  HistoryMap histmap_;

  rosbag2_cpp::Reader reader_;

  MessageDecoder<tobas_msgs::msg::Imu> imu_decoder_;
  MessageDecoder<tobas_msgs::msg::OdometryWithCovarianceStamped> odom_cov_decoder_;
  MessageDecoder<tobas_msgs::msg::MagneticField> mag_decoder_;
  MessageDecoder<tobas_msgs::msg::Gnss> gnss_decoder_;
  MessageDecoder<tobas_msgs::msg::RCInput> rcin_decoder_;
  MessageDecoder<tobas_msgs::msg::Battery> battery_decoder_;
  MessageDecoder<tobas_msgs::msg::Cpu> cpu_decoder_;
  MessageDecoder<tobas_msgs::msg::RotorStateArray> rotor_states_decoder_;
  MessageDecoder<tobas_msgs::msg::RotorSpeedArray> rotor_speeds_decoder_;
  MessageDecoder<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_decoder_;
  MessageDecoder<tobas_msgs::msg::Latency> latency_decoder_;
  MessageDecoder<tobas_msgs::msg::VibrationLevel> vibe_decoder_;
  MessageDecoder<tobas_msgs::msg::RepulsiveAcceleration> repulsive_accel_decoder_;
  MessageDecoder<tobas_kdl_msgs::msg::WrenchStamped> wrench_decoder_;
  MessageDecoder<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb_decoder_;
  MessageDecoder<tobas_debug_msgs::msg::MulticopterControllerFeedback> mr_ctrl_fb_decoder_;

  bool openRosBag(const std::string& path);

  bool rotorLinkNamesValid(const tobas_msgs::msg::RotorStateArray& msg);
  bool rotorLinkNamesValid(const tobas_msgs::msg::RotorSpeedArray& msg);

  std::string makeCsvHeader() const;
  std::string makeCsvDataRow(Time time, const SerializedDataMap& data);

  bool exportOldestImuLine(std::ofstream& file, Time before_this_time = INT64_MAX);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
