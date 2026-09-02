// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/logs_gcs/exporter/export_thread_csv.hpp"

#include <ranges>

#include <QDebug>

#include <tobas_constants/path.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_kdl/rotation.hpp>
#include <tobas_ros2_tools/rosbag.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_string_tools/chars.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
ExportThreadCsv::ExportThreadCsv(const QString& log_name, const QString& save_path)
  : log_name_(log_name), save_path_(save_path)
{
}

void ExportThreadCsv::run()
{
  // Open rosbag.
  const auto log_path = ros2::expandUser(kRosbagDirHome) / log_name_.toStdString();
  if (!openRosBag(log_path.string())) {
    if (!ros2::reindexRosBag(log_path.string())) {
      Q_EMIT finished(false, "The log file is broken and failed to fix it.");
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
  csv_file << makeCsvHeader() << std::endl;

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
      else if (topic.ends_with(str::concat('/', topic::kRepulsiveAccel).data())) {
        const auto& msg = repulsive_accel_decoder_.decode(ser_data);
        histmap_[ros2::nanoseconds(msg.header.stamp)][topic::kRepulsiveAccel] = ser_data;
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

bool ExportThreadCsv::openRosBag(const std::string& path)
{
  try {
    reader_.open(path);
  }
  catch (const std::exception& e) {
    qWarning() << "Failed to open" << QString::fromStdString(path) + ":" << e.what();
    return false;
  }

  return true;
}

bool ExportThreadCsv::rotorLinkNamesValid(const tobas_msgs::msg::RotorStateArray& msg)
{
  if (msg.states.size() != rotor_link_names_.size()) {
    return false;
  }

  for (const auto [elem, link_name] : std::views::zip(msg.states, rotor_link_names_)) {
    if (elem.link_name != link_name) {
      return false;
    }
  }

  return true;
}

bool ExportThreadCsv::rotorLinkNamesValid(const tobas_msgs::msg::RotorSpeedArray& msg)
{
  if (msg.speeds.size() != rotor_link_names_.size()) {
    return false;
  }

  for (const auto [elem, link_name] : std::views::zip(msg.speeds, rotor_link_names_)) {
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
    csv_header += "Roter/TargetRPM/" + link_name + ',';
  }
  for (const auto& link_name : rotor_link_names_) {
    csv_header += "Roter/CurrentRPM/" + link_name + ',';
  }
  for (const auto& link_name : rotor_link_names_) {
    csv_header += "Roter/Link/" + link_name + ',';
  }

  csv_header += "Latency/ControlLatency[us],"
                "VibrationLevel/X[m/s^2],VibrationLevel/Y[m/s^2],VibrationLevel/Z[m/s^2],"
                "RepulsiveAccel/X[m/s^2],RepulsiveAccel/Y[m/s^2],RepulsiveAccel/Z[m/s^2],"
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

std::string ExportThreadCsv::makeCsvDataRow(Time time, const SerializedDataMap& data)
{
  std::string res;

  // Time
  res += std::to_string(time * 1e-9) + ',';

  // Pose, Twist, Accel
  const auto odom_it = data.find(topic::kOdometry);
  if (odom_it != data.end()) {
    const auto& msg = odom_cov_decoder_.decode(odom_it->second);

    const auto& pos = msg.odom.odom.frame.trans;
    const kdl::Rotation rot(msg.odom.odom.frame.rot.data);
    const auto [roll, pitch, yaw] = rot.getRPY();
    res += std::to_string(pos.x) + ',' + std::to_string(pos.y) + ',' + std::to_string(pos.z) + ',' +
           std::to_string(st::rad2deg(roll)) + ',' + std::to_string(st::rad2deg(pitch)) + ',' +
           std::to_string(st::rad2deg(yaw)) + ',';

    const auto& lin_vel = msg.odom.odom.twist.linear;
    const auto& ang_vel = msg.odom.odom.twist.angular;
    res += std::to_string(lin_vel.x) + ',' + std::to_string(lin_vel.y) + ',' + std::to_string(lin_vel.z) + ',' +
           std::to_string(ang_vel.x) + ',' + std::to_string(ang_vel.y) + ',' + std::to_string(ang_vel.z) + ',';

    const auto& lin_acc = msg.odom.odom.accel.linear;
    const auto& ang_acc = msg.odom.odom.accel.angular;
    res += std::to_string(lin_acc.x) + ',' + std::to_string(lin_acc.y) + ',' + std::to_string(lin_acc.z) + ',' +
           std::to_string(ang_acc.x) + ',' + std::to_string(ang_acc.y) + ',' + std::to_string(ang_acc.z) + ',';
  }
  else {
    res += std::string(18, ',');
  }

  // IMU
  const auto imu_raw_it = data.find(topic::kImuRaw);
  if (imu_raw_it != data.end()) {
    const auto& msg = imu_decoder_.decode(imu_raw_it->second);
    const auto& accel = msg.accel;
    const auto& gyro = msg.gyro;
    const auto& dgyro = msg.dgyro;
    res += std::to_string(accel.x) + ',' + std::to_string(accel.y) + ',' + std::to_string(accel.z) + ',' +
           std::to_string(gyro.x) + ',' + std::to_string(gyro.y) + ',' + std::to_string(gyro.z) + ',' +
           std::to_string(dgyro.x) + ',' + std::to_string(dgyro.y) + ',' + std::to_string(dgyro.z) + ',';
  }
  else {
    res += std::string(9, ',');
  }

  // Magnetic Field
  const auto mag_it = data.find(topic::kMagneticField);
  if (mag_it != data.end()) {
    const auto& msg = mag_decoder_.decode(mag_it->second);
    const auto& mag = msg.mag;
    res += std::to_string(mag.x) + ',' + std::to_string(mag.y) + ',' + std::to_string(mag.z) + ',';
  }
  else {
    res += ",,,";
  }

  // GNSS
  const auto gnss_it = data.find(topic::kGnss);
  if (gnss_it != data.end()) {
    const auto& msg = gnss_decoder_.decode(gnss_it->second);
    if (msg.fix_type == tobas_msgs::msg::Gnss::FIX_3D) {
      res += std::to_string(msg.latitude) + ',' + std::to_string(msg.longitude) + ',' + std::to_string(msg.height_msl) +
             ',' + std::to_string(msg.ground_speed.x) + ',' + std::to_string(msg.ground_speed.y) + ',' +
             std::to_string(msg.ground_speed.z) + ',';
    }
    else {
      res += std::string(6, ',');
    }
  }
  else {
    res += std::string(6, ',');
  }

  // RC Input
  const auto rcin_it = data.find(topic::kRcInput);
  if (rcin_it != data.end()) {
    const auto& msg = rcin_decoder_.decode(rcin_it->second);
    if (msg.status == tobas_msgs::msg::RCInput::STATUS_OK) {
      res += std::to_string(msg.roll) + ',' + std::to_string(msg.pitch) + ',' + std::to_string(msg.throttle) + ',' +
             std::to_string(msg.yaw) + ',' + std::to_string(msg.mode) + ',' + std::to_string(msg.sub_mode) + ',' +
             std::to_string(msg.enable) + ',' + std::to_string(msg.kill) + ',';
    }
    else {
      res += std::string(8, ',');
    }
  }
  else {
    res += std::string(8, ',');
  }

  // Battery
  const auto batt_it = data.find(topic::kBattery);
  if (batt_it != data.end()) {
    const auto& msg = battery_decoder_.decode(batt_it->second);
    res += std::to_string(msg.voltage) + ',' + std::to_string(msg.current) + ',';
  }
  else {
    res += ",,";
  }

  // Engine
  const auto ice_cmd_it = data.find(topic::kIcePropulsionSystemCmd);
  if (ice_cmd_it != data.end()) {
    const auto& msg = ice_cmd_decoder_.decode(ice_cmd_it->second);
    res += std::to_string(msg.engine_throttle * 100) + ',';
  }
  else {
    res += ",";
  }

  // CPU
  const auto cpu_it = data.find(topic::kCpu);
  if (cpu_it != data.end()) {
    const auto& msg = cpu_decoder_.decode(cpu_it->second);
    res += std::to_string(msg.frequency) + ',' + std::to_string(msg.temperature) + ',' +
           std::to_string(msg.load * 100) + ',';
  }
  else {
    res += ",,,";
  }

  // Target Rotor Speeds
  const auto rotor_speeds_cmd_it = data.find(topic::kRotorSpeedsCmd);
  if (rotor_speeds_cmd_it != data.end()) {
    const auto& msg = rotor_speeds_decoder_.decode(rotor_speeds_cmd_it->second);
    for (const auto& elem : msg.speeds) {
      res += std::to_string(st::rps2rpm(elem.speed)) + ',';
    }
  }
  else {
    res += std::string(rotor_link_names_.size(), ',');
  }

  // Rotor States
  const auto rotor_states_it = data.find(topic::kRotorStates);
  if (rotor_states_it != data.end()) {
    const auto& msg = rotor_states_decoder_.decode(rotor_states_it->second);
    for (const auto& elem : msg.states) {
      res += std::to_string(st::rps2rpm(elem.speed)) + ',';
    }
    for (const auto& elem : msg.states) {
      const auto comm_ok = (elem.status != tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE);
      res += std::to_string(static_cast<int>(comm_ok)) + ',';
    }
  }
  else {
    res += std::string(rotor_link_names_.size() * 2, ',');
  }

  // Control Latency
  const auto ctrl_latency_it = data.find(topic::kControlLatency);
  if (ctrl_latency_it != data.end()) {
    const auto& msg = latency_decoder_.decode(ctrl_latency_it->second);
    res += std::to_string(ros2::microseconds(msg.data)) + ',';
  }
  else {
    res += ",";
  }

  // Vibration Level
  const auto vibe_it = data.find(topic::kVibrationLevel);
  if (vibe_it != data.end()) {
    const auto& msg = vibe_decoder_.decode(vibe_it->second);
    const auto& vibe = msg.data;
    res += std::to_string(vibe.x) + ',' + std::to_string(vibe.y) + ',' + std::to_string(vibe.z) + ',';
  }
  else {
    res += ",,,";
  }

  // Repulsive Acceleration
  const auto repulsive_accel_it = data.find(topic::kRepulsiveAccel);
  if (repulsive_accel_it != data.end()) {
    const auto& msg = repulsive_accel_decoder_.decode(repulsive_accel_it->second);
    const auto& accel = msg.accel;
    res += std::to_string(accel.x) + ',' + std::to_string(accel.y) + ',' + std::to_string(accel.z) + ',';
  }
  else {
    res += ",,,";
  }

  // Disturbance Force
  const auto dist_force_it = data.find(topic::kDisturbanceForce);
  if (dist_force_it != data.end()) {
    const auto& msg = wrench_decoder_.decode(dist_force_it->second);
    const auto& force = msg.wrench.force;
    const auto& torque = msg.wrench.torque;
    res += std::to_string(force.x) + ',' + std::to_string(force.y) + ',' + std::to_string(force.z) + ',' +
           std::to_string(torque.x) + ',' + std::to_string(torque.y) + ',' + std::to_string(torque.z) + ',';
  }
  else {
    res += std::string(6, ',');
  }

  // Observer Feedback
  const auto obsv_fb_it = data.find(topic::kObsvFeedback);
  if (obsv_fb_it != data.end()) {
    const auto& msg = obsv_fb_decoder_.decode(obsv_fb_it->second);
    const auto& ab = msg.accel_bias;
    const auto& gb = msg.gyro_bias;
    const auto& mhb = msg.mag_hard_bias;
    const auto& msb = msg.mag_soft_bias;
    res += std::to_string(ab.data[0]) + ',' + std::to_string(ab.data[1]) + ',' + std::to_string(ab.data[2]) + ',' +
           std::to_string(gb.data[0]) + ',' + std::to_string(gb.data[1]) + ',' + std::to_string(gb.data[2]) + ',' +
           std::to_string(mhb.data[0]) + ',' + std::to_string(mhb.data[1]) + ',' + std::to_string(mhb.data[2]) + ',' +
           std::to_string(msb.data[0]) + ',' + std::to_string(msb.data[4]) + ',' + std::to_string(msb.data[8]) + ',' +
           std::to_string(msb.data[1]) + ',' + std::to_string(msb.data[5]) + ',' + std::to_string(msb.data[2]) + ',' +
           std::to_string(msg.baro_alt_bias) + ',' + std::to_string(msg.gravity) + ',' +
           std::to_string(msg.gnss_anomaly_score) + ',';
  }
  else {
    res += std::string(18, ',');
  }

  // Multirotor Controller Feedback
  const auto mr_ctrl_fb_it = data.find(topic::kMRCtrlFeedback);
  if (mr_ctrl_fb_it != data.end()) {
    const auto& msg = mr_ctrl_fb_decoder_.decode(mr_ctrl_fb_it->second);
    const auto& pos_ie = msg.position_integral_error;
    const auto& rot_ie = msg.angle_integral_error;
    res += std::to_string(pos_ie.x) + ',' + std::to_string(pos_ie.y) + ',' + std::to_string(pos_ie.z) + ',' +
           std::to_string(rot_ie.x) + ',' + std::to_string(rot_ie.y) + ',' + std::to_string(rot_ie.z);  // End
  }
  else {
    res += std::string(6 - 1, ',');
  }

  return res;
}

bool ExportThreadCsv::exportOldestImuLine(std::ofstream& file, Time before_this_time)
{
  // Check whether any IMU data older than the time threshold exists.
  Time imu_time = -1;
  for (auto it = histmap_.begin(); it != histmap_.end() && it->first < before_this_time; ++it) {
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
  file << makeCsvDataRow(imu_time, line_data) << std::endl;

  return true;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
