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

std::string ExportThreadCsv::makeCsvDataRow(Time _time, const SerializedDataMap& _data)
{
  constexpr std::size_t kInitialRowCapacity = 2048;  // Estimated bytes for a typical row.
  std::string res;
  res.reserve(kInitialRowCapacity);

  const auto append_vector3 = [&res](const auto& _v)
  { std::format_to(std::back_inserter(res), "{},{},{},", _v.x, _v.y, _v.z); };

  const auto append_odometry = [&res, &append_vector3](const auto& _odom)
  {
    const kdl::Rotation rot(_odom.frame.rot.data);
    const auto [roll, pitch, yaw] = rot.getRPY();

    append_vector3(_odom.frame.trans);
    std::format_to(std::back_inserter(res), "{},{},{},", st::rad2deg(roll), st::rad2deg(pitch), st::rad2deg(yaw));
    append_vector3(_odom.twist.linear);
    append_vector3(_odom.twist.angular);
    append_vector3(_odom.accel.linear);
    append_vector3(_odom.accel.angular);
  };

  // Time
  std::format_to(std::back_inserter(res), "{},", _time * 1e-9);

  // Odometry
  const auto odom_it = _data.find(topic::kOdometry);
  if (odom_it != _data.end()) {
    const auto& msg = odom_cov_decoder_.decode(odom_it->second);
    append_odometry(msg.odom.odom);
  }
  else {
    res.append(18, ',');
  }

  // Target Odometry
  const auto traj_setpoint_it = _data.find(topic::kTrajSetpoint);
  if (traj_setpoint_it != _data.end()) {
    const auto& msg = odom_decoder_.decode(traj_setpoint_it->second);
    append_odometry(msg.odom);
  }
  else {
    res.append(18, ',');
  }

  // IMU
  const auto imu_raw_it = _data.find(topic::kImuRaw);
  if (imu_raw_it != _data.end()) {
    const auto& msg = imu_decoder_.decode(imu_raw_it->second);
    append_vector3(msg.accel);
    append_vector3(msg.gyro);
    append_vector3(msg.dgyro);
  }
  else {
    res.append(9, ',');
  }

  // Magnetic Field
  const auto mag_it = _data.find(topic::kMagneticField);
  if (mag_it != _data.end()) {
    const auto& msg = mag_decoder_.decode(mag_it->second);
    append_vector3(msg.mag);
  }
  else {
    res.append(3, ',');
  }

  // GNSS
  const auto gnss_it = _data.find(topic::kGnss);
  if (gnss_it != _data.end()) {
    const auto& msg = gnss_decoder_.decode(gnss_it->second);
    if (msg.fix_type == tobas_msgs::msg::Gnss::FIX_3D) {
      std::format_to(std::back_inserter(res), "{},{},{},", msg.latitude, msg.longitude, msg.height_msl);
      append_vector3(msg.ground_speed);
    }
    else {
      res.append(6, ',');
    }
  }
  else {
    res.append(6, ',');
  }

  // RC Input
  const auto rcin_it = _data.find(topic::kRcInput);
  if (rcin_it != _data.end()) {
    const auto& msg = rcin_decoder_.decode(rcin_it->second);
    if (msg.status == tobas_msgs::msg::RCInput::STATUS_OK) {
      std::format_to(
        std::back_inserter(res),
        "{},{},{},{},{:d},{:d},{:d},{:d},",
        msg.roll,
        msg.pitch,
        msg.throttle,
        msg.yaw,
        msg.mode,
        msg.sub_mode,
        msg.enable,
        msg.kill);
    }
    else {
      res.append(8, ',');
    }
  }
  else {
    res.append(8, ',');
  }

  // Battery
  const auto batt_it = _data.find(topic::kBattery);
  if (batt_it != _data.end()) {
    const auto& msg = battery_decoder_.decode(batt_it->second);
    std::format_to(std::back_inserter(res), "{},{},", msg.voltage, msg.current);
  }
  else {
    res.append(2, ',');
  }

  // Engine
  const auto ice_cmd_it = _data.find(topic::kIcePropulsionSystemCmd);
  if (ice_cmd_it != _data.end()) {
    const auto& msg = ice_cmd_decoder_.decode(ice_cmd_it->second);
    std::format_to(std::back_inserter(res), "{},", msg.engine_throttle * 100);
  }
  else {
    res.append(1, ',');
  }

  // CPU
  const auto cpu_it = _data.find(topic::kCpu);
  if (cpu_it != _data.end()) {
    const auto& msg = cpu_decoder_.decode(cpu_it->second);
    std::format_to(std::back_inserter(res), "{:d},{},{},", msg.frequency, msg.temperature, msg.load * 100);
  }
  else {
    res.append(3, ',');
  }

  // Target Rotor Speeds
  const auto rotor_speeds_cmd_it = _data.find(topic::kRotorSpeedsCmd);
  if (rotor_speeds_cmd_it != _data.end()) {
    const auto& msg = rotor_speeds_decoder_.decode(rotor_speeds_cmd_it->second);
    for (const auto& elem : msg.speeds) {
      std::format_to(std::back_inserter(res), "{},", st::rps2rpm(elem.speed));
    }
  }
  else {
    res.append(rotor_link_names_.size(), ',');
  }

  // Rotor States
  const auto rotor_states_it = _data.find(topic::kRotorStates);
  if (rotor_states_it != _data.end()) {
    const auto& msg = rotor_states_decoder_.decode(rotor_states_it->second);
    for (const auto& elem : msg.states) {
      std::format_to(std::back_inserter(res), "{},", st::rps2rpm(elem.speed));
    }
    for (const auto& elem : msg.states) {
      const auto comm_ok = (elem.status != tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE);
      std::format_to(std::back_inserter(res), "{:d},", comm_ok);
    }
  }
  else {
    res.append(rotor_link_names_.size() * 2, ',');
  }

  // Control Latency
  const auto ctrl_latency_it = _data.find(topic::kControlLatency);
  if (ctrl_latency_it != _data.end()) {
    const auto& msg = latency_decoder_.decode(ctrl_latency_it->second);
    std::format_to(std::back_inserter(res), "{},", ros2::microseconds(msg.data));
  }
  else {
    res.append(1, ',');
  }

  // Vibration Level
  const auto vibe_it = _data.find(topic::kVibrationLevel);
  if (vibe_it != _data.end()) {
    const auto& msg = vibe_decoder_.decode(vibe_it->second);
    append_vector3(msg.data);
  }
  else {
    res.append(3, ',');
  }

  // Additional Acceleration
  const auto additional_accel_it = _data.find(topic::kAdditionalAccel);
  if (additional_accel_it != _data.end()) {
    const auto& msg = accel_decoder_.decode(additional_accel_it->second);
    append_vector3(msg.accel.linear);
  }
  else {
    res.append(3, ',');
  }

  // Disturbance Force
  const auto dist_force_it = _data.find(topic::kDisturbanceForce);
  if (dist_force_it != _data.end()) {
    const auto& msg = wrench_decoder_.decode(dist_force_it->second);
    append_vector3(msg.wrench.force);
    append_vector3(msg.wrench.torque);
  }
  else {
    res.append(6, ',');
  }

  // Observer Feedback
  const auto obsv_fb_it = _data.find(topic::kObsvFeedback);
  if (obsv_fb_it != _data.end()) {
    const auto& msg = obsv_fb_decoder_.decode(obsv_fb_it->second);
    const auto& ab = msg.accel_bias;
    const auto& gb = msg.gyro_bias;
    const auto& mhb = msg.mag_hard_bias;
    const auto& msb = msg.mag_soft_bias;
    std::format_to(std::back_inserter(res), "{},{},{},", ab.data[0], ab.data[1], ab.data[2]);
    std::format_to(std::back_inserter(res), "{},{},{},", gb.data[0], gb.data[1], gb.data[2]);
    std::format_to(std::back_inserter(res), "{},{},{},", mhb.data[0], mhb.data[1], mhb.data[2]);
    std::format_to(std::back_inserter(res), "{},{},{},", msb.data[0], msb.data[4], msb.data[8]);
    std::format_to(std::back_inserter(res), "{},{},{},", msb.data[1], msb.data[5], msb.data[2]);
    std::format_to(std::back_inserter(res), "{},{},{},", msg.baro_alt_bias, msg.gravity, msg.gnss_anomaly_score);
  }
  else {
    res.append(18, ',');
  }

  // Multirotor Controller Feedback
  const auto mr_ctrl_fb_it = _data.find(topic::kMRCtrlFeedback);
  if (mr_ctrl_fb_it != _data.end()) {
    const auto& msg = mr_ctrl_fb_decoder_.decode(mr_ctrl_fb_it->second);
    append_vector3(msg.position_integral_error);
    const auto& rot_ie = msg.angle_integral_error;
    std::format_to(std::back_inserter(res), "{},{},{}", rot_ie.x, rot_ie.y, rot_ie.z);  // End
  }
  else {
    res.append(6 - 1, ',');
  }

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
