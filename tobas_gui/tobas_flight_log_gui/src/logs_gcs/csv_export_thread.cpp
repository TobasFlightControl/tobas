#include "tobas_flight_log_gui/logs_gcs/csv_export_thread.hpp"

#include <fstream>
#include <ranges>

#include <QDebug>

#include <tobas_constants/constants.hpp>
#include <tobas_kdl/rotation.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/rosbag.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace gui
{
namespace log
{
namespace
{
/* メッセージが更新されていたら文字列として出力する． */
template <typename MsgType, typename Func>
std::string updateAndExportString(
  const std::shared_ptr<MsgType>& cur_ptr,
  std::shared_ptr<MsgType>& last_ptr,
  const std::string& empty_str,
  Func formatter)
{
  if (cur_ptr && cur_ptr != last_ptr) {
    last_ptr = cur_ptr;
    return formatter(cur_ptr);
  }
  return empty_str;
}
}  // namespace

CsvExportThread::CsvExportThread(const QString& log_name, const QString& save_path)
  : log_name_(log_name), save_path_(save_path)
{
}

void CsvExportThread::run()
{
  // Open rosbag
  const auto log_path = ros2::expandUser(tobas::kRosbagDirHome) / log_name_.toStdString();
  if (!open(log_path.string())) {
    if (!ros2::reindexRosBag(log_path.string())) {
      Q_EMIT finished(false, "The log file is broken and failed to fix it.");
      return;
    }
    if (!open(log_path.string())) {
      Q_EMIT finished(false, "Failed to open the log file. The data is probably corrupted.");
      return;
    }
  }

  // Get the rotor link names
  while (reader_.has_next()) {
    const auto msg = reader_.read_next();
    const rclcpp::SerializedMessage ser_msg(*msg->serialized_data);
    if (msg->topic_name.ends_with(path::join("/", tobas::kRotorStatesTopic))) {
      try {
        const auto rotor_states = rotor_states_decoder_.decode(msg->recv_timestamp, ser_msg);
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

  // Reset the reader
  reader_.seek(0);

  // Create CSV header
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
    "GNSS/Latitude[deg],GNSS/Longitude[deg],GNSS/Altitude[m],"
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
                "DisturbanceForce/Force/X[N],DisturbanceForce/Force/Y[N],DisturbanceForce/Force/Z[N],"
                "DisturbanceForce/Torque/X[Nm],DisturbanceForce/Torque/Y[Nm],DisturbanceForce/Torque/Z[Nm],"
                "Observer/AccelBias/X[m/s^2],Observer/AccelBias/Y[m/s^2],Observer/AccelBias/Z[m/s^2],"
                "Observer/GyroBias/X[rad/s],Observer/GyroBias/Y[rad/s],Observer/GyroBias/Z[rad/s],"
                "Observer/MagHardIronBias/X[-],Observer/MagHardIronBias/Y[-],Observer/MagHardIronBias/Z[-],"
                "Observer/MagSoftIronBias/XX[-],Observer/MagSoftIronBias/YY[-],Observer/MagSoftIronBias/ZZ[-],"
                "Observer/MagSoftIronBias/XY[-],Observer/MagSoftIronBias/YZ[-],Observer/MagSoftIronBias/ZX[-],"
                "Observer/Gravity[m/s^2],Observer/GNSSAnomalyScore,"
                "MultirotorController/IntegralError/X[m*s],"
                "MultirotorController/IntegralError/Y[m*s],"
                "MultirotorController/IntegralError/Z[m*s],"
                "MultirotorController/IntegralError/Roll[rad*s],"
                "MultirotorController/IntegralError/Pitch[rad*s],"
                "MultirotorController/IntegralError/Yaw[rad*s]\n";

  std::ofstream csv_file(save_path_.toStdString());
  if (!csv_file.is_open()) {
    finished(false, "Failed to open " + save_path_);
    return;
  }

  csv_file << csv_header;

  rcutils_time_point_value_t start_time = 0;  // [ns]
  bool is_timer_started = false;
  constexpr rcutils_time_point_value_t kTimeThreshold = 1'000'000;  // [ns]

  const auto& metadata = reader_.get_metadata();
  const auto record_start_time = metadata.starting_time.time_since_epoch().count();
  reader_.seek(record_start_time);

  while (reader_.has_next()) {
    const auto msg = reader_.read_next();
    const auto& cur_time = msg->recv_timestamp;  // [ns]
    const rclcpp::SerializedMessage ser_msg(*msg->serialized_data);

    if (is_timer_started && cur_time - start_time > kTimeThreshold) {
      is_timer_started = false;
      csv_file << makeCsvRow(cur_time);
    }

    try {
      if (msg->topic_name.ends_with(path::join("/", tobas::kImuRawTopic))) {
        cur_data_.imu = std::make_shared<tobas_msgs::msg::Imu>(imu_decoder_.decode(cur_time, ser_msg));
        if (!is_timer_started) {
          is_timer_started = true;
          start_time = msg->recv_timestamp;
        }
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kOdometryTopic))) {
        cur_data_.odom = std::make_shared<tobas_msgs::msg::Odometry>(odom_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kMagTopic))) {
        cur_data_.mag = std::make_shared<tobas_msgs::msg::MagneticField>(mag_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kGnssTopic))) {
        cur_data_.gnss = std::make_shared<tobas_msgs::msg::Gnss>(gnss_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kRcInputTopic))) {
        cur_data_.rcin = std::make_shared<tobas_msgs::msg::RCInput>(rcin_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kBatteryTopic))) {
        cur_data_.battery = std::make_shared<tobas_msgs::msg::Battery>(battery_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kCpuTopic))) {
        cur_data_.cpu = std::make_shared<tobas_msgs::msg::Cpu>(cpu_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kRotorStatesTopic))) {
        cur_data_.rotor_states =
          std::make_shared<tobas_msgs::msg::RotorStateArray>(rotor_states_decoder_.decode(cur_time, ser_msg));
        if (!rotorLinkNamesValid(cur_data_.rotor_states)) {
          csv_file.close();
          Q_EMIT finished(false, "Rotor link names mismatch.");
          return;
        }
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kRotorSpeedsCmdTopic))) {
        cur_data_.rotor_speeds =
          std::make_shared<tobas_msgs::msg::RotorSpeedArray>(rotor_speeds_decoder_.decode(cur_time, ser_msg));
        if (!rotorLinkNamesValid(cur_data_.rotor_speeds)) {
          csv_file.close();
          Q_EMIT finished(false, "Rotor link names mismatch.");
          return;
        }
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kIcePropulsionSystemCmdTopic))) {
        cur_data_.ice_cmd =
          std::make_shared<tobas_msgs::msg::IcePropulsionSystemCommand>(ice_cmd_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kControlLatencyTopic))) {
        cur_data_.latency = std::make_shared<tobas_msgs::msg::Latency>(latency_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kVibrationLevelTopic))) {
        cur_data_.vibration_level =
          std::make_shared<tobas_msgs::msg::VibrationLevel>(vibe_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kDisturbanceForceTopic))) {
        cur_data_.disturbance_force =
          std::make_shared<tobas_kdl_msgs::msg::WrenchStamped>(wrench_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kObsvFeedbackTopic))) {
        cur_data_.obsv_fb =
          std::make_shared<tobas_debug_msgs::msg::ObserverFeedback>(obsv_fb_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kMRCtrlFeedbackTopic))) {
        cur_data_.mr_ctrl_fb = std::make_shared<tobas_debug_msgs::msg::MulticopterControllerFeedback>(
          mr_ctrl_fb_decoder_.decode(cur_time, ser_msg));
      }
    }
    catch (const std::exception& e) {
      csv_file.close();
      Q_EMIT finished(false, "Failed to deserialize \"" + QString::fromStdString(msg->topic_name) + "\".");
      return;
    }
  }

  csv_file.close();
  Q_EMIT finished(true, "");
}

bool CsvExportThread::open(const std::string& rosbag_path)
{
  try {
    reader_.open(rosbag_path);
  }
  catch (const std::exception& e) {
    qWarning() << "Failed to open " << QString::fromStdString(rosbag_path) + ": " << e.what();
    return false;
  }

  return true;
}

bool CsvExportThread::rotorLinkNamesValid(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg)
{
  if (msg->states.size() != rotor_link_names_.size()) {
    return false;
  }

  for (const auto [elem, link_name] : std::views::zip(msg->states, rotor_link_names_)) {
    if (elem.link_name != link_name) {
      return false;
    }
  }

  return true;
}

bool CsvExportThread::rotorLinkNamesValid(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& msg)
{
  if (msg->speeds.size() != rotor_link_names_.size()) {
    return false;
  }

  for (const auto [elem, link_name] : std::views::zip(msg->speeds, rotor_link_names_)) {
    if (elem.link_name != link_name) {
      return false;
    }
  }

  return true;
}

std::string CsvExportThread::makeCsvRow(const rcutils_time_point_value_t& cur_time)
{
  std::string csv_line;

  // Time
  csv_line += std::to_string(cur_time * 1e-9) + ',';

  // Pose, Twist, Accel
  csv_line += updateAndExportString(
    cur_data_.odom,
    last_data_.odom,
    std::string(18, ','),
    [](const auto& msg)
    {
      const auto& pos = msg->frame.trans;
      const kdl::Rotation rot(msg->frame.rot.data);
      const auto [roll, pitch, yaw] = rot.getRPY();
      const auto pose_str = std::to_string(pos.x) + ',' + std::to_string(pos.y) + ',' + std::to_string(pos.z) + ',' +
                            std::to_string(tbs::rad2deg(roll)) + ',' + std::to_string(tbs::rad2deg(pitch)) + ',' +
                            std::to_string(tbs::rad2deg(yaw)) + ',';

      const auto& lin_vel = msg->twist.linear;
      const auto& ang_vel = msg->twist.angular;
      const auto twist_str = std::to_string(lin_vel.x) + ',' + std::to_string(lin_vel.y) + ',' +
                             std::to_string(lin_vel.z) + ',' + std::to_string(ang_vel.x) + ',' +
                             std::to_string(ang_vel.y) + ',' + std::to_string(ang_vel.z) + ',';

      const auto& lin_acc = msg->accel.linear;
      const auto& ang_acc = msg->accel.angular;
      const auto accel_str = std::to_string(lin_acc.x) + ',' + std::to_string(lin_acc.y) + ',' +
                             std::to_string(lin_acc.z) + ',' + std::to_string(ang_acc.x) + ',' +
                             std::to_string(ang_acc.y) + ',' + std::to_string(ang_acc.z) + ',';

      return pose_str + twist_str + accel_str;
    });

  // IMU
  csv_line += updateAndExportString(
    cur_data_.imu,
    last_data_.imu,
    std::string(9, ','),
    [](const auto& msg)
    {
      const auto& accel = msg->accel;
      const auto& gyro = msg->gyro;
      const auto& dgyro = msg->dgyro;
      return std::to_string(accel.x) + ',' + std::to_string(accel.y) + ',' + std::to_string(accel.z) + ',' +
             std::to_string(gyro.x) + ',' + std::to_string(gyro.y) + ',' + std::to_string(gyro.z) + ',' +
             std::to_string(dgyro.x) + ',' + std::to_string(dgyro.y) + ',' + std::to_string(dgyro.z) + ',';
    });

  // Magnetic Field
  csv_line += updateAndExportString(
    cur_data_.mag,
    last_data_.mag,
    ",,,",
    [](const auto& msg)
    {
      const auto& mag = msg->mag;
      return std::to_string(mag.x) + ',' + std::to_string(mag.y) + ',' + std::to_string(mag.z) + ',';
    });

  // GNSS
  csv_line += updateAndExportString(
    cur_data_.gnss,
    last_data_.gnss,
    std::string(6, ','),
    [](const auto& msg)
    {
      return std::to_string(msg->latitude) + ',' + std::to_string(msg->longitude) + ',' +
             std::to_string(msg->altitude) + ',' + std::to_string(msg->ground_speed.x) + ',' +
             std::to_string(msg->ground_speed.y) + ',' + std::to_string(msg->ground_speed.z) + ',';
    });

  // RC Input
  csv_line += updateAndExportString(
    cur_data_.rcin,
    last_data_.rcin,
    std::string(8, ','),
    [](const auto& msg)
    {
      return std::to_string(msg->roll) + ',' + std::to_string(msg->pitch) + ',' + std::to_string(msg->throttle) + ',' +
             std::to_string(msg->yaw) + ',' + std::to_string(msg->mode) + ',' + std::to_string(msg->sub_mode) + ',' +
             std::to_string(msg->enable) + ',' + std::to_string(msg->kill) + ',';
    });

  // Battery
  csv_line += updateAndExportString(
    cur_data_.battery,
    last_data_.battery,
    ",,",
    [](const auto& msg) { return std::to_string(msg->voltage) + ',' + std::to_string(msg->current) + ','; });

  // Engine
  csv_line += updateAndExportString(
    cur_data_.ice_cmd,
    last_data_.ice_cmd,
    ",",
    [](const auto& msg) { return std::to_string(msg->engine_throttle * 100) + ','; });

  // CPU
  csv_line += updateAndExportString(
    cur_data_.cpu,
    last_data_.cpu,
    ",,,",
    [](const auto& msg)
    {
      return std::to_string(msg->frequency) + ',' + std::to_string(msg->temperature) + ',' +
             std::to_string(msg->load * 100) + ',';
    });

  // Target Rotor Speeds
  csv_line += updateAndExportString(
    cur_data_.rotor_speeds,
    last_data_.rotor_speeds,
    std::string(rotor_link_names_.size(), ','),
    [](const auto& msg)
    {
      std::string res;
      for (const auto& elem : msg->speeds) {
        res += std::to_string(tbs::rps2rpm(elem.speed)) + ',';
      }
      return res;
    });

  // Rotor States
  csv_line += updateAndExportString(
    cur_data_.rotor_states,
    last_data_.rotor_states,
    std::string(rotor_link_names_.size() * 2, ','),
    [](const auto& msg)
    {
      std::string res;
      for (const auto& elem : msg->states) {
        res += std::to_string(tbs::rps2rpm(elem.speed)) + ',';
      }
      for (const auto& elem : msg->states) {
        const auto comm_ok = (elem.status != tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE);
        res += std::to_string(static_cast<int>(comm_ok)) + ',';
      }
      return res;
    });

  // Control Latency
  csv_line += updateAndExportString(
    cur_data_.latency,
    last_data_.latency,
    ",",
    [](const auto& msg) { return std::to_string(ros2::microseconds(msg->data)) + ','; });

  // Vibration Level
  csv_line += updateAndExportString(
    cur_data_.vibration_level,
    last_data_.vibration_level,
    ",,,",
    [](const auto& msg)
    {
      const auto& vib = msg->data;
      return std::to_string(vib.x) + ',' + std::to_string(vib.y) + ',' + std::to_string(vib.z) + ',';
    });

  // Disturbance Force
  csv_line += updateAndExportString(
    cur_data_.disturbance_force,
    last_data_.disturbance_force,
    std::string(6, ','),
    [](const auto& msg)
    {
      const auto& force = msg->wrench.force;
      const auto& torque = msg->wrench.torque;
      return std::to_string(force.x) + ',' + std::to_string(force.y) + ',' + std::to_string(force.z) + ',' +
             std::to_string(torque.x) + ',' + std::to_string(torque.y) + ',' + std::to_string(torque.z) + ',';
    });

  // Observer Feedback
  csv_line += updateAndExportString(
    cur_data_.obsv_fb,
    last_data_.obsv_fb,
    std::string(17, ','),
    [](const auto& msg)
    {
      const auto& ab = msg->accel_bias;
      const auto& gb = msg->gyro_bias;
      const auto& mhb = msg->mag_hard_bias;
      const auto& msb = msg->mag_soft_bias;
      return std::to_string(ab.data[0]) + ',' + std::to_string(ab.data[1]) + ',' + std::to_string(ab.data[2]) + ',' +
             std::to_string(gb.data[0]) + ',' + std::to_string(gb.data[1]) + ',' + std::to_string(gb.data[2]) + ',' +
             std::to_string(mhb.data[0]) + ',' + std::to_string(mhb.data[1]) + ',' + std::to_string(mhb.data[2]) + ',' +
             std::to_string(msb.data[0]) + ',' + std::to_string(msb.data[4]) + ',' + std::to_string(msb.data[8]) + ',' +
             std::to_string(msb.data[1]) + ',' + std::to_string(msb.data[5]) + ',' + std::to_string(msb.data[2]) + ',' +
             std::to_string(msg->gravity) + ',' + std::to_string(msg->gnss_anomaly_score) + ',';
    });

  // Multirotor Controller Feedback
  csv_line += updateAndExportString(
    cur_data_.mr_ctrl_fb,
    last_data_.mr_ctrl_fb,
    std::string(6 - 1, ','),
    [](const auto& msg)
    {
      const auto& pos_ie = msg->position_integral_error;
      const auto& rot_ie = msg->angle_integral_error;
      return std::to_string(pos_ie.x) + ',' + std::to_string(pos_ie.y) + ',' + std::to_string(pos_ie.z) + ',' +
             std::to_string(rot_ie.x) + ',' + std::to_string(rot_ie.y) + ',' + std::to_string(rot_ie.z);  // End
    });

  return csv_line + '\n';
}
}  // namespace log
}  // namespace gui
