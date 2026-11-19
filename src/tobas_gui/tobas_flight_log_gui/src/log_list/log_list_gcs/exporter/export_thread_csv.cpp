// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_list/log_list_gcs/exporter/export_thread_csv.hpp"

#include <cstddef>
#include <format>
#include <iterator>
#include <ranges>

#include <QDebug>

#include <tobas_constants/path.hpp>
#include <tobas_constants/ros.hpp>
#include <tobas_kdl/rotation.hpp>
#include <tobas_ros2_tools/rosbag.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_string_tools/chars.hpp>

namespace tobas::gui::log
{
ExportThreadCsv::ExportThreadCsv(const QString& _log_name, const QString& _save_path)
  : log_name_(_log_name), save_path_(_save_path)
{
}

void ExportThreadCsv::run()
{
  // Open rosbag.
  const auto log_path = ros2::expandUser(kRosbagDirHome) / log_name_.toStdString();
  if (!openRosBag(log_path.string())) {
    if (!ros2::reindexRosBag(log_path.string())) {
      Q_EMIT finished(false, "Failed to repair the log file.");
      return;
    }
    if (!openRosBag(log_path.string())) {
      Q_EMIT finished(false, "Failed to open the log file. The data is probably corrupted.");
      return;
    }
  }

  // Get the rotor link names.
  rotor_link_names_.clear();
  reader_.seek(0);
  while (reader_.has_next()) {
    const auto bag_msg = reader_.read_next();
    if (bag_msg->topic_name.ends_with(str::concat('/', topic::kRotorStates).data())) {
      try {
        const auto rotor_states = rotor_states_decoder_.decode(bag_msg->serialized_data);
        for (const auto& elem : rotor_states.states) {
          rotor_link_names_.push_back(elem.link_name);
        }
        qInfo() << "The number of rotors:" << rotor_link_names_.size();
      }
      catch (const std::exception& e) {
        Q_EMIT finished(false, "Failed to decode the first rotor states message.");
        return;
      }
      break;
    }
  }

  // Open the output file.
  std::ofstream csv_file(save_path_.toStdString());
  if (!csv_file.is_open()) {
    finished(false, "Failed to open " + save_path_);
    return;
  }

  // Write header.
  csv_file << makeCsvHeader() << '\n';

  // Read and export data.
  histmap_.clear();
  reader_.seek(0);
  while (reader_.has_next()) {
    const auto bag_msg = reader_.read_next();

    const auto& ser_data = bag_msg->serialized_data;
    const auto& topic = bag_msg->topic_name;

    // Save messages to the history map.
    try {
      if (topic.ends_with(str::concat('/', topic::kImuRaw).data())) {
        const auto& msg = imu_decoder_.decode(ser_data);
        const auto cur_time = ros2::nanoseconds(msg.header.stamp);
        histmap_[cur_time][topic::kImuRaw] = ser_data;

        // Write old data incrementally to prevent the history map from growing too large.
        constexpr rcutils_time_point_value_t kExpirationTime = 1'000'000'000;  // [ns]
        exportOldestImuLine(csv_file, cur_time - kExpirationTime);
      }
      else if (topic.ends_with(str::concat('/', topic::kOdometry).data())) {
        const auto& msg = odom_cov_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kOdometry] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kTrajSetpoint).data())) {
        const auto& msg = odom_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kTrajSetpoint] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kMagneticField).data())) {
        const auto& msg = mag_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kMagneticField] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kGnss).data())) {
        const auto& msg = gnss_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kGnss] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kRcInput).data())) {
        const auto& msg = rcin_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kRcInput] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kBattery).data())) {
        const auto& msg = battery_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kBattery] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kCpu).data())) {
        const auto& msg = cpu_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kCpu] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kRotorStates).data())) {
        const auto& msg = rotor_states_decoder_.decode(ser_data);
        if (!rotorLinkNamesValid(msg)) {
          csv_file.close();
          Q_EMIT finished(false, "Rotor link names mismatch.");
          return;
        }
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kRotorStates] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kRotorSpeedsCmd).data())) {
        const auto& msg = rotor_speeds_decoder_.decode(ser_data);
        if (!rotorLinkNamesValid(msg)) {
          csv_file.close();
          Q_EMIT finished(false, "Rotor link names mismatch.");
          return;
        }
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kRotorSpeedsCmd] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kIcePropulsionSystemCmd).data())) {
        const auto& msg = ice_cmd_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kIcePropulsionSystemCmd] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kControlLatency).data())) {
        const auto& msg = latency_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kControlLatency] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kVibrationLevel).data())) {
        const auto& msg = vibe_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kVibrationLevel] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kAdditionalAccel).data())) {
        const auto& msg = accel_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kAdditionalAccel] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kDisturbanceForce).data())) {
        const auto& msg = wrench_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kDisturbanceForce] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kObsvFeedback).data())) {
        const auto& msg = obsv_fb_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kObsvFeedback] = ser_data;
      }
      else if (topic.ends_with(str::concat('/', topic::kMRCtrlFeedback).data())) {
        const auto& msg = mr_ctrl_fb_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kMRCtrlFeedback] = ser_data;
      }
    }
    catch (const std::exception& e) {
      csv_file.close();
      Q_EMIT finished(false, "Failed to deserialize '" + QString::fromStdString(topic) + "'.");
      return;
    }
  }

  // Write all remaining data.
  while (exportOldestImuLine(csv_file)) {}

  csv_file.close();
  Q_EMIT finished(true, "");
}

bool ExportThreadCsv::openRosBag(const std::string& _path)
{
  try {
    reader_.open(_path);
  }
  catch (const std::exception& e) {
    qWarning() << "Failed to open" << QString::fromStdString(_path) + ":" << e.what();
    return false;
  }

  return true;
}

bool ExportThreadCsv::rotorLinkNamesValid(const tobas_msgs::msg::RotorStateArray& _msg)
{
  if (_msg.states.size() != rotor_link_names_.size()) {
    return false;
  }

  for (const auto [elem, link_name] : std::views::zip(_msg.states, rotor_link_names_)) {
    if (elem.link_name != link_name) {
      return false;
    }
  }

  return true;
}

bool ExportThreadCsv::rotorLinkNamesValid(const tobas_msgs::msg::RotorSpeedArray& _msg)
{
  if (_msg.speeds.size() != rotor_link_names_.size()) {
    return false;
  }

  for (const auto [elem, link_name] : std::views::zip(_msg.speeds, rotor_link_names_)) {
    if (elem.link_name != link_name) {
      return false;
    }
  }

  return true;
}

std::string ExportThreadCsv::makeCsvHeader() const
{
  std::string csv_header =
    "Time[s],"
    "Pose/X[m],Pose/Y[m],Pose/Z[m],"
    "Pose/Roll[deg],Pose/Pitch[deg],Pose/Yaw[deg],"
    "Twist/LinearVelocity/X[m/s],Twist/LinearVelocity/Y[m/s],Twist/LinearVelocity/Z[m/s],"
    "Twist/AngularVelocity/X[rad/s],Twist/AngularVelocity/Y[rad/s],Twist/AngularVelocity/Z[rad/s],"
    "Accel/LinearAccel/X[m/s^2],Accel/LinearAccel/Y[m/s^2],Accel/LinearAccel/Z[m/s^2],"
    "Accel/AngularAccel/X[rad/s^2],Accel/AngularAccel/Y[rad/s^2],Accel/AngularAccel/Z[rad/s^2],"
    "TargetPose/X[m],TargetPose/Y[m],TargetPose/Z[m],"
    "TargetPose/Roll[deg],TargetPose/Pitch[deg],TargetPose/Yaw[deg],"
    "TargetTwist/LinearVelocity/X[m/s],TargetTwist/LinearVelocity/Y[m/s],TargetTwist/LinearVelocity/Z[m/s],"
    "TargetTwist/AngularVelocity/X[rad/s],TargetTwist/AngularVelocity/Y[rad/s],TargetTwist/AngularVelocity/Z[rad/s],"
    "TargetAccel/LinearAccel/X[m/s^2],TargetAccel/LinearAccel/Y[m/s^2],TargetAccel/LinearAccel/Z[m/s^2],"
    "TargetAccel/AngularAccel/X[rad/s^2],TargetAccel/AngularAccel/Y[rad/s^2],TargetAccel/AngularAccel/Z[rad/s^2],"
    "IMU/Accel/X[m/s^2],IMU/Accel/Y[m/s^2],IMU/Accel/Z[m/s^2],"
    "IMU/Gyro/X[rad/s],IMU/Gyro/Y[rad/s],IMU/Gyro/Z[rad/s],"
    "IMU/DGyro/X[rad/s^2],IMU/DGyro/Y[rad/s^2],IMU/DGyro/Z[rad/s^2],"
    "MagneticField/X[-],MagneticField/Y[-],MagneticField/Z[-],"
    "GNSS/Latitude[deg],GNSS/Longitude[deg],GNSS/HeightMSL[m],"
    "GNSS/EastSpeed[m/s],GNSS/NorthSpeed[m/s],GNSS/UpSpeed[m/s],"
    "RCInput/Roll,RCInput/Pitch,RCInput/Throttle,RCInput/Yaw,"
    "RCInput/FlightMode,RCInput/SubMode,RCInput/Enable,RCInput/Kill,"
    "Battery/Voltage[V],Battery/Current[A],"
    "Engine/Throttle[%],"
    "CPU/Frequency[GHz],CPU/Temperature[degC],CPU/Load[%],";

  for (const auto& link_name : rotor_link_names_) {
    csv_header += "Rotor/TargetRPM/" + link_name + ',';
  }
  for (const auto& link_name : rotor_link_names_) {
    csv_header += "Rotor/CurrentRPM/" + link_name + ',';
  }
  for (const auto& link_name : rotor_link_names_) {
    csv_header += "Rotor/Link/" + link_name + ',';
  }

  csv_header += "Latency/ControlLatency[us],"
                "VibrationLevel/X[m/s^2],VibrationLevel/Y[m/s^2],VibrationLevel/Z[m/s^2],"
                "AdditionalAccel/X[m/s^2],AdditionalAccel/Y[m/s^2],AdditionalAccel/Z[m/s^2],"
                "DisturbanceForce/Force/X[N],DisturbanceForce/Force/Y[N],DisturbanceForce/Force/Z[N],"
                "DisturbanceForce/Torque/X[Nm],DisturbanceForce/Torque/Y[Nm],DisturbanceForce/Torque/Z[Nm],"
                "Observer/AccelBias/X[m/s^2],Observer/AccelBias/Y[m/s^2],Observer/AccelBias/Z[m/s^2],"
                "Observer/GyroBias/X[rad/s],Observer/GyroBias/Y[rad/s],Observer/GyroBias/Z[rad/s],"
                "Observer/MagHardIronBias/X[-],Observer/MagHardIronBias/Y[-],Observer/MagHardIronBias/Z[-],"
                "Observer/MagSoftIronBias/XX[-],Observer/MagSoftIronBias/YY[-],Observer/MagSoftIronBias/ZZ[-],"
                "Observer/MagSoftIronBias/XY[-],Observer/MagSoftIronBias/YZ[-],Observer/MagSoftIronBias/ZX[-],"
                "Observer/BarometerAltitudeBias[m],Observer/Gravity[m/s^2],Observer/GNSSAnomalyScore,"
                "MultirotorController/IntegralError/X[m*s],"
                "MultirotorController/IntegralError/Y[m*s],"
                "MultirotorController/IntegralError/Z[m*s],"
                "MultirotorController/IntegralError/Roll[rad*s],"
                "MultirotorController/IntegralError/Pitch[rad*s],"
                "MultirotorController/IntegralError/Yaw[rad*s]";

  return csv_header;
}

const std::string& ExportThreadCsv::makeCsvDataRow(Time _time, const SerializedDataMap& _data)
{
  constexpr std::size_t kInitialRowCapacity = 2048;  // Estimated bytes for a typical row.
  auto& res = csv_row_buffer_;
  res.clear();
  res.reserve(kInitialRowCapacity);

  // Limit measurement precision to keep exported CSV files compact.
  const auto append_vector3 = [&res](const auto& _v)
  { std::format_to(std::back_inserter(res), "{:.6g},{:.6g},{:.6g},", _v.x, _v.y, _v.z); };

  const auto append_odometry = [&res, &append_vector3](const auto& _odom)
  {
    const kdl::Rotation rot(_odom.frame.rot.data);
    const auto [roll, pitch, yaw] = rot.getRPY();

    append_vector3(_odom.frame.trans);
    std::format_to(
      std::back_inserter(res), "{:.6g},{:.6g},{:.6g},", st::rad2deg(roll), st::rad2deg(pitch), st::rad2deg(yaw));
    append_vector3(_odom.twist.linear);
    append_vector3(_odom.twist.angular);
    append_vector3(_odom.accel.linear);
    append_vector3(_odom.accel.angular);
  };

  const auto append_message =
    [&_data, &res](const auto& _topic, auto& _decoder, std::size_t _column_count, const auto& _append)
  {
    const auto it = _data.find(_topic);
    if (it == _data.end()) {
      res.append(_column_count, ',');
      return;
    }
    _append(_decoder.decode(it->second));
  };

  // Time
  std::format_to(std::back_inserter(res), "{:.6f},", _time * 1e-9);

  // Odometry
  append_message(topic::kOdometry, odom_cov_decoder_, 18, [&](const auto& _msg) { append_odometry(_msg.odom.odom); });

  // Target Odometry
  append_message(topic::kTrajSetpoint, odom_decoder_, 18, [&](const auto& _msg) { append_odometry(_msg.odom); });

  // IMU
  append_message(
    topic::kImuRaw,
    imu_decoder_,
    9,
    [&](const auto& _msg)
    {
      append_vector3(_msg.accel);
      append_vector3(_msg.gyro);
      append_vector3(_msg.dgyro);
    });

  // Magnetic Field
  append_message(topic::kMagneticField, mag_decoder_, 3, [&](const auto& _msg) { append_vector3(_msg.mag); });

  // GNSS
  append_message(
    topic::kGnss,
    gnss_decoder_,
    6,
    [&](const auto& _msg)
    {
      if (_msg.fix_type == tobas_msgs::msg::Gnss::FIX_3D) {
        std::format_to(std::back_inserter(res), "{:.8f},{:.8f},{:.6g},", _msg.latitude, _msg.longitude, _msg.height_msl);
        append_vector3(_msg.ground_speed);
      }
      else {
        res.append(6, ',');
      }
    });

  // RC Input
  append_message(
    topic::kRcInput,
    rcin_decoder_,
    8,
    [&](const auto& _msg)
    {
      if (_msg.status == tobas_msgs::msg::RCInput::STATUS_OK) {
        std::format_to(
          std::back_inserter(res),
          "{:.6g},{:.6g},{:.6g},{:.6g},{:d},{:d},{:d},{:d},",
          _msg.roll,
          _msg.pitch,
          _msg.throttle,
          _msg.yaw,
          _msg.mode,
          _msg.sub_mode,
          _msg.enable,
          _msg.kill);
      }
      else {
        res.append(8, ',');
      }
    });

  // Battery
  append_message(
    topic::kBattery,
    battery_decoder_,
    2,
    [&](const auto& _msg) { std::format_to(std::back_inserter(res), "{:.6g},{:.6g},", _msg.voltage, _msg.current); });

  // Engine
  append_message(
    topic::kIcePropulsionSystemCmd,
    ice_cmd_decoder_,
    1,
    [&](const auto& _msg) { std::format_to(std::back_inserter(res), "{:.6g},", _msg.engine_throttle * 100); });

  // CPU
  append_message(
    topic::kCpu,
    cpu_decoder_,
    3,
    [&](const auto& _msg)
    {
      std::format_to(std::back_inserter(res), "{:d},{:.6g},{:.6g},", _msg.frequency, _msg.temperature, _msg.load * 100);
    });

  // Target Rotor Speeds
  append_message(
    topic::kRotorSpeedsCmd,
    rotor_speeds_decoder_,
    rotor_link_names_.size(),
    [&](const auto& _msg)
    {
      for (const auto& elem : _msg.speeds) {
        std::format_to(std::back_inserter(res), "{:.6g},", st::rps2rpm(elem.speed));
      }
    });

  // Rotor States
  append_message(
    topic::kRotorStates,
    rotor_states_decoder_,
    rotor_link_names_.size() * 2,
    [&](const auto& _msg)
    {
      for (const auto& elem : _msg.states) {
        std::format_to(std::back_inserter(res), "{:.6g},", st::rps2rpm(elem.speed));
      }
      for (const auto& elem : _msg.states) {
        const auto comm_ok = (elem.status != tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE);
        std::format_to(std::back_inserter(res), "{:d},", comm_ok);
      }
    });

  // Control Latency
  append_message(
    topic::kControlLatency,
    latency_decoder_,
    1,
    [&](const auto& _msg) { std::format_to(std::back_inserter(res), "{:.6g},", ros2::microseconds(_msg.data)); });

  // Vibration Level
  append_message(topic::kVibrationLevel, vibe_decoder_, 3, [&](const auto& _msg) { append_vector3(_msg.data); });

  // Additional Acceleration
  append_message(
    topic::kAdditionalAccel, accel_decoder_, 3, [&](const auto& _msg) { append_vector3(_msg.accel.linear); });

  // Disturbance Force
  append_message(
    topic::kDisturbanceForce,
    wrench_decoder_,
    6,
    [&](const auto& _msg)
    {
      append_vector3(_msg.wrench.force);
      append_vector3(_msg.wrench.torque);
    });

  // Observer Feedback
  append_message(
    topic::kObsvFeedback,
    obsv_fb_decoder_,
    18,
    [&](const auto& _msg)
    {
      const auto& ab = _msg.accel_bias;
      const auto& gb = _msg.gyro_bias;
      const auto& mhb = _msg.mag_hard_bias;
      const auto& msb = _msg.mag_soft_bias;
      std::format_to(std::back_inserter(res), "{:.6g},{:.6g},{:.6g},", ab.data[0], ab.data[1], ab.data[2]);
      std::format_to(std::back_inserter(res), "{:.6g},{:.6g},{:.6g},", gb.data[0], gb.data[1], gb.data[2]);
      std::format_to(std::back_inserter(res), "{:.6g},{:.6g},{:.6g},", mhb.data[0], mhb.data[1], mhb.data[2]);
      std::format_to(std::back_inserter(res), "{:.6g},{:.6g},{:.6g},", msb.data[0], msb.data[4], msb.data[8]);
      std::format_to(std::back_inserter(res), "{:.6g},{:.6g},{:.6g},", msb.data[1], msb.data[5], msb.data[2]);
      std::format_to(
        std::back_inserter(res), "{:.6g},{:.6g},{:.6g},", _msg.baro_alt_bias, _msg.gravity, _msg.gnss_anomaly_score);
    });

  // Multirotor Controller Feedback
  append_message(
    topic::kMRCtrlFeedback,
    mr_ctrl_fb_decoder_,
    6,
    [&](const auto& _msg)
    {
      append_vector3(_msg.position_integral_error);
      append_vector3(_msg.angle_integral_error);
    });

  res.pop_back();  // Remove the final column separator.
  return res;
}

bool ExportThreadCsv::exportOldestImuLine(std::ofstream& _file, Time _before_this_time)
{
  // Check whether any IMU data older than the time threshold exists.
  Time imu_time = -1;
  for (auto it = histmap_.begin(); it != histmap_.end() && it->first < _before_this_time; ++it) {
    if (it->second.contains(topic::kImuRaw)) {
      imu_time = it->first;
      break;
    }
  }
  if (imu_time < 0) {
    return false;
  }

  // Aggregate data to write while removing it from the history map.
  SerializedDataMap line_data;
  auto it = histmap_.begin();
  while (it != histmap_.end() && it->first <= imu_time) {
    for (const auto& [topic, ser_data] : it->second) {
      line_data[topic] = ser_data;
    }
    it = histmap_.erase(it);
  }

  // Write data to one row.
  _file << makeCsvDataRow(imu_time, line_data) << '\n';

  return true;
}
}  // namespace tobas::gui::log
