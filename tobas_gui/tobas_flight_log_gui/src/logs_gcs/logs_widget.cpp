#include "tobas_flight_log_gui/logs_gcs/logs_widget.hpp"

#include <fstream>
#include <iostream>
#include <string>

#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <rosbag2_cpp/reindexer.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_kdl/rotation.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_flight_log_gui/constants.hpp"
#include "tobas_flight_log_gui/logs_gcs/log_item.hpp"

#include <rosbag2_storage/storage_filter.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace log
{
void CsvExportWorker::process(const QString& log_name, const QString& savePath)
{
  // open rosbag
  const auto log_path = ros2::expandUser(tobas::kRosbagDirHome) / log_name.toStdString();
  if (!open(log_path.string())) {
    if (!reindex(log_path.string())) {
      return;
    }
    if (!open(log_path.string())) {
      return;
    }
  }

  // get roter num
  num_rotors_ = 0;
  try {
    while (reader_.has_next()) {
      const auto msg = reader_.read_next();
      rclcpp::SerializedMessage ser_msg(*msg->serialized_data);

      if (msg->topic_name.ends_with(path::join("/", tobas::kRotorStatesTopic))) {
        std::shared_ptr<tobas_msgs::msg::RotorStateArray> temp_rotor_states_array =
          std::make_shared<tobas_msgs::msg::RotorStateArray>(rotor_states_decoder_.decode(msg->recv_timestamp, ser_msg));
        num_rotors_ = (int8_t)temp_rotor_states_array->states.size();
        break;
      }
    }
  }
  catch (const std::exception& e) {
    Q_EMIT error(QString::fromStdString(e.what()));
    return;
  }
  reader_.seek(0);

  // write header
  exportCsvHeader = "time,\
    Pose/CurrentX[m], Pose/currently[m], Pose/CurrentZ[m],\
    Pose/CurrentRoll[deg], Pose/CurrentPitch[deg], Pose/CurrentYaw[deg],\
    Twist/CurrentLinearVelocityX[m/s], Twist/CurrentLinearVelocityY[m/s], Twist/CurrentLinearVelocityZ[m/s],\
    Twist/CurrentAngularVelocityX[rad/s], Twist/CurrentAngularVelocityY[rad/s], Twist/CurrentAngularVelocityZ[rad/s],\
    Accel/CurrentLinearAccelX[m/s^2], Accel/CurrentLinearAccelY[m/s^2], Accel/CurrentLinearAccelZ[m/s^2],\
    Accel/CurrentAngularAccelX[rad/s^2], Accel/CurrentAngularAccelY[rad/s^2], Accel/CurrentAngularAccelZ[rad/s^2],\
    IMU/accel/x[m/s^2], IMU/accel/y[m/s^2], IMU/accel/z[m/s^2],\
    IMU/gyro/x[rad/s], IMU/gyro/y[rad/s], IMU/gyro/z[rad/s],\
    IMU/dgyro/x[rad/s^2], IMU/dgyro/y[rad/s^2], IMU/dgyro/z[rad/s^2],\
    MagneticField/X[-], MagneticField/Y[-], MagneticField/Z[-],\
    GNSS/latitude[deg], GNSS/longitude[deg], GNSS/altitude[m],\
    GNSS/EastSpeed[m/s], GNSS/NorthSpeed[m/s], GNSS/UpSpeed[m/s],\
    RCInput/Roll, RCInput/Pitch, RCInput/Throttle, RCInput/Yaw,\
    RCInput/FlightMode, RCInput/SubMode, RCInput/Enable, RCInput/kill,\
    Battery/voltage[V], Battery/current[A],\
    EngineThrottle[%],\
    CPU/Frequency[GHz], CPU/Temperature[degC], CPU/Load[%],";

  std::string tmp_header = "";
  for (int i = 0; i < (int8_t)num_rotors_; i++) {
    tmp_header += "RoterSpeed/TargetRPM(Propeller_" + std::to_string(i) + "),";
  }
  for (int8_t i = 0; i < (int8_t)num_rotors_; i++) {
    tmp_header += "RoterSpeed/CurrentRPM(Propeller_" + std::to_string(i) + "),";
  }
  for (int i = 0; i < (int8_t)num_rotors_; i++) {
    tmp_header += "RoterLink/CommunicationState(Propeller_" + std::to_string(i) + "),";
  }
  exportCsvHeader += tmp_header;

  exportCsvHeader += "Latency/IMUSamplingTime[us], ControlLatency[us],\
    VibrationLevel/X[m/s^2], VibrationLevel/Y[m/s^2], VibrationLevel/Z[m/s^2],\
    DisturbanceForce/ForceX[N], DisturbanceForce/ForceY[N], DisturbanceForce/ForceZ[N],\
    DisturbanceForce/TorqueX[N], DisturbanceForce/TorqueY[N], DisturbanceForce/TorqueZ[N],\
    Observer/AccelBiasX[m/s²], Observer/AccelBiasY[m/s²], Observer/AccelBiasZ[m/s²],\
    Observer/GyroBiasX[rad/s], Observer/GyroBiasY[rad/s], Observer/GyroBiasZ[rad/s],\
    Observer/MagHard-IronBiasX, Observer/MagHard-IronBiasY, Observer/MagHard-IronBiasZ,\
    Observer/MagSoft-IronBiasXX, Observer/MagSoft-IronBiasYY, Observer/MagSoft-IronBiasZZ,\
    Observer/MagSoft-IronBiasXY, Observer/MagSoft-IronBiasYZ, Observer/MagSoft-IronBiasZX,\
    Observer/Gravity, Observer/GNSSAnomalyScore,\
    MultirotorController/XIntegralError[m*s], MultirotorController/YIntegralError[m*s], MultirotorController/ZIntegralError[m*s],\
    MultirotorController/RollIntegralError[rad*s], MultirotorController/PitchIntegralError[rad*s], MultirotorController/YawIntegralError[rad*s]";

  exportCsvHeader += "\n";

  try {
    std::ofstream csvFile(savePath.toStdString());
    if (!csvFile.is_open()) {
      std::cerr << "Couldn't write to csv file: " << savePath.toStdString() << std::endl;
      return;
    }
    csvFile << CsvExportWorker::exportCsvHeader;

    rcutils_time_point_value_t start_time = 0;
    bool is_timer_started = false;
    const int64_t TIME_THRESHOLD_NS = 1 * 1000 * 1000;

    const auto& metadata = reader_.get_metadata();
    const auto record_start_time = metadata.starting_time.time_since_epoch().count();
    reader_.seek(record_start_time);
    while (reader_.has_next()) {
      const auto msg = reader_.read_next();
      const auto& cur_time = msg->recv_timestamp;  // [ns]
      if (cur_time - start_time > TIME_THRESHOLD_NS && is_timer_started) {
        is_timer_started = false;
        csvFile << makeCsvRow(cur_time);
      }

      rclcpp::SerializedMessage ser_msg(*msg->serialized_data);

      if (msg->topic_name.ends_with(path::join("/", tobas::kImuRawTopic))) {
        curData_.imu = std::make_shared<tobas_msgs::msg::Imu>(imu_decoder_.decode(cur_time, ser_msg));
        is_timer_started = true;
        start_time = msg->recv_timestamp;
        // csvFile << makeCsvRow(cur_time);
      }
      else {
        if (msg->topic_name.ends_with(path::join("/", tobas::kOdometryTopic))) {
          curData_.odom = std::make_shared<tobas_msgs::msg::Odometry>(odom_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kImuFiltTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kMagTopic))) {
          curData_.mag = std::make_shared<tobas_msgs::msg::MagneticField>(mag_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kGnssTopic))) {
          curData_.gnss = std::make_shared<tobas_msgs::msg::Gnss>(gnss_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kRcInputTopic))) {
          curData_.rcin = std::make_shared<tobas_msgs::msg::RCInput>(rcin_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kBatteryTopic))) {
          curData_.battery = std::make_shared<tobas_msgs::msg::Battery>(battery_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kCpuTopic))) {
          curData_.cpu = std::make_shared<tobas_msgs::msg::Cpu>(cpu_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kRotorStatesTopic))) {
          curData_.rotor_states =
            std::make_shared<tobas_msgs::msg::RotorStateArray>(rotor_states_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kRotorSpeedsCmdTopic))) {
          curData_.rotor_speeds =
            std::make_shared<tobas_msgs::msg::RotorSpeedArray>(rotor_speeds_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kJointStatesTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kJointPosCmdTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kJointVelCmdTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kJointEffCmdTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kIcePropulsionSystemCmdTopic))) {
          curData_.ice_cmd =
            std::make_shared<tobas_msgs::msg::IcePropulsionSystemCommand>(ice_cmd_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kImuSamplingTimeTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kControlLatencyTopic))) {
          curData_.latency = std::make_shared<tobas_msgs::msg::Latency>(latency_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kVibrationLevelTopic))) {
          curData_.vibration_level =
            std::make_shared<tobas_msgs::msg::VibrationLevel>(vibe_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kDisturbanceForceTopic))) {
          curData_.disturbance_force =
            std::make_shared<tobas_kdl_msgs::msg::WrenchStamped>(wrench_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kObsvFeedbackTopic))) {
          curData_.obsv_fb =
            std::make_shared<tobas_debug_msgs::msg::ObserverFeedback>(obsv_fb_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kMRCtrlFeedbackTopic))) {
          curData_.mr_ctrl_fb = std::make_shared<tobas_debug_msgs::msg::MulticopterControllerFeedback>(
            mr_ctrl_fb_decoder_.decode(cur_time, ser_msg));
        }
      }
    }
    csvFile.close();
  }
  catch (const std::exception& e) {
    Q_EMIT error(QString::fromStdString(e.what()));
    return;
  }
  Q_EMIT finished();
}

bool CsvExportWorker::reindex(const std::string& rosbag_path)
{
  rosbag2_cpp::Reindexer reindexer;

  rosbag2_storage::StorageOptions options;
  options.uri = rosbag_path;
  options.storage_id = "mcap";

  try {
    reindexer.reindex(options);
  }
  catch (const std::exception& e) {
    qWarning() << "Failed to reindex " << QString::fromStdString(rosbag_path) + ": " << e.what();
    return false;
  }

  return true;
}

bool CsvExportWorker::open(const std::string& rosbag_path)
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

template <typename T, typename Func>
std::string CsvExportWorker::getLogString(
  const std::shared_ptr<T>& cur_ptr,
  std::shared_ptr<T>& last_ptr,
  const std::string& empty_str,
  Func formatter)
{
  if (cur_ptr != nullptr && cur_ptr != last_ptr) {
    last_ptr = cur_ptr;
    return formatter(cur_ptr);
  }
  return empty_str;
}

std::string CsvExportWorker::makeCsvRow(const auto& cur_time)
{
  std::string csv_line = std::to_string(cur_time / 1000000000.0) + ",";

  // Pose, Twist, Accel
  std::string odom_empty_str(18, ',');
  csv_line += getLogString(
    curData_.odom,
    lastData_.odom,
    odom_empty_str,
    [](const auto& msg)
    {
      const auto& pos = msg->frame.trans;
      const kdl::Rotation rot(msg->frame.rot.data);
      const auto [roll, pitch, yaw] = rot.getRPY();
      std::string pose_str = std::to_string(pos.x) + "," + std::to_string(pos.y) + "," + std::to_string(pos.z) + "," +
                             std::to_string(roll) + "," + std::to_string(pitch) + "," + std::to_string(yaw) + ",";

      const auto& lin_vel = msg->twist.linear;
      const auto& ang_vel = msg->twist.angular;
      std::string twist_str = std::to_string(lin_vel.x) + "," + std::to_string(lin_vel.y) + "," +
                              std::to_string(lin_vel.z) + "," + std::to_string(ang_vel.x) + "," +
                              std::to_string(ang_vel.y) + "," + std::to_string(ang_vel.z) + ",";

      const auto& lin_acc = msg->accel.linear;
      const auto& ang_acc = msg->accel.angular;
      std::string accel_str = std::to_string(lin_acc.x) + "," + std::to_string(lin_acc.y) + "," +
                              std::to_string(lin_acc.z) + "," + std::to_string(ang_acc.x) + "," +
                              std::to_string(ang_acc.y) + "," + std::to_string(ang_acc.z) + ",";

      return pose_str + twist_str + accel_str;
    });

  // Imu
  const auto& imu_accel = curData_.imu->accel;
  const auto& imu_gyro = curData_.imu->gyro;
  const auto& imu_dgyro = curData_.imu->dgyro;
  csv_line += std::to_string(imu_accel.x) + "," + std::to_string(imu_accel.y) + "," + std::to_string(imu_accel.z) +
              "," + std::to_string(imu_gyro.x) + "," + std::to_string(imu_gyro.y) + "," + std::to_string(imu_gyro.z) +
              "," + std::to_string(imu_dgyro.x) + "," + std::to_string(imu_dgyro.y) + "," +
              std::to_string(imu_dgyro.z) + ",";

  // Magnetic_Field
  csv_line += getLogString(
    curData_.mag,
    lastData_.mag,
    ",,,",
    [](const auto& msg)
    { return std::to_string(msg->mag.x) + "," + std::to_string(msg->mag.y) + "," + std::to_string(msg->mag.z) + ","; });

  // Gnss
  std::string gnss_empty_str(6, ',');
  csv_line += getLogString(
    curData_.gnss,
    lastData_.gnss,
    gnss_empty_str,
    [](const auto& msg)
    {
      return std::to_string(msg->latitude) + "," + std::to_string(msg->longitude) + "," +
             std::to_string(msg->altitude) + "," + std::to_string(msg->ground_speed.x) + "," +
             std::to_string(msg->ground_speed.y) + "," + std::to_string(msg->ground_speed.z) + ",";
    });

  // RC Input
  std::string rcin_empty_str(8, ',');
  csv_line += getLogString(
    curData_.rcin,
    lastData_.rcin,
    rcin_empty_str,
    [](const auto& msg)
    {
      return std::to_string(msg->roll) + "," + std::to_string(msg->pitch) + "," + std::to_string(msg->throttle) + "," +
             std::to_string(msg->yaw) + "," + std::to_string(msg->mode) + "," + std::to_string(msg->sub_mode) + "," +
             std::to_string(msg->enable) + "," + std::to_string(msg->kill) + ",";
    });

  // Battery
  csv_line += getLogString(
    curData_.battery,
    lastData_.battery,
    ",,",
    [](const auto& msg) { return std::to_string(msg->voltage) + "," + std::to_string(msg->current) + ","; });

  // Engine
  csv_line += getLogString(
    curData_.ice_cmd,
    lastData_.ice_cmd,
    ",",
    [](const auto& msg) { return std::to_string(msg->engine_throttle) + ","; });

  // CPU
  csv_line += getLogString(
    curData_.cpu,
    lastData_.cpu,
    ",,,",
    [](const auto& msg)
    {
      return std::to_string(msg->frequency) + "," + std::to_string(msg->temperature) + "," + std::to_string(msg->load) +
             ",";
    });

  // Rotor Speed
  // Rotor Link
  csv_line += getLogString(
    curData_.rotor_speeds,
    lastData_.rotor_speeds,
    ",,,,",
    [](const auto& msg)
    {
      std::string temp_str = "";
      for (const auto& elem : msg->speeds) {
        temp_str += std::to_string(elem.speed) + ",";
      }
      return temp_str;
    });

  std::string rotor_empty_str(8, ',');
  csv_line += getLogString(
    curData_.rotor_states,
    lastData_.rotor_states,
    rotor_empty_str,
    [](const auto& msg)
    {
      std::string temp_str = "";
      for (const auto& elem : msg->states) {
        temp_str += std::to_string(elem.speed) + ",";
      }
      for (const auto& elem : msg->states) {
        temp_str +=
          std::to_string(static_cast<int>(elem.status != tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE)) + ",";
      }
      return temp_str;
    });

  // Latency
  csv_line += getLogString(
    curData_.latency,
    lastData_.latency,
    ",,",
    [](const auto& msg) { return std::to_string(ros2::microseconds(msg->data)) + ",,"; });

  // Vibration_Level
  csv_line += getLogString(
    curData_.vibration_level,
    lastData_.vibration_level,
    ",,,",
    [](const auto& msg) {
      return std::to_string(msg->data.x) + "," + std::to_string(msg->data.y) + "," + std::to_string(msg->data.z) + ",";
    });

  // Disturbance_Force
  std::string dist_force_empty_str(6, ',');
  csv_line += getLogString(
    curData_.disturbance_force,
    lastData_.disturbance_force,
    dist_force_empty_str,
    [](const auto& msg)
    {
      return std::to_string(msg->wrench.force.x) + "," + std::to_string(msg->wrench.force.y) + "," +
             std::to_string(msg->wrench.force.z) + "," + std::to_string(msg->wrench.torque.x) + "," +
             std::to_string(msg->wrench.torque.y) + "," + std::to_string(msg->wrench.torque.z) + ",";
    });

  // Observer
  std::string obsv_force_empty_str(17, ',');
  csv_line += getLogString(
    curData_.obsv_fb,
    lastData_.obsv_fb,
    obsv_force_empty_str,
    [](const auto& msg)
    {
      return std::to_string(msg->accel_bias.data[0]) + "," + std::to_string(msg->accel_bias.data[1]) + "," +
             std::to_string(msg->accel_bias.data[2]) + "," + std::to_string(msg->gyro_bias.data[0]) + "," +
             std::to_string(msg->gyro_bias.data[1]) + "," + std::to_string(msg->gyro_bias.data[2]) + "," +
             std::to_string(msg->mag_hard_bias.data[0]) + "," + std::to_string(msg->mag_hard_bias.data[1]) + "," +
             std::to_string(msg->mag_hard_bias.data[2]) + "," + std::to_string(msg->mag_soft_bias.data[0]) + "," +
             std::to_string(msg->mag_soft_bias.data[4]) + "," + std::to_string(msg->mag_hard_bias.data[8]) + "," +
             std::to_string(msg->mag_soft_bias.data[1]) + "," + std::to_string(msg->mag_soft_bias.data[5]) + "," +
             std::to_string(msg->mag_hard_bias.data[2]) + "," + std::to_string(msg->gravity) + "," +
             std::to_string(msg->gnss_anomaly_score);
    });

  // Multirotor_Controller
  std::string mr_ctrl_fb_empty_str(6, ',');
  csv_line += getLogString(
    curData_.mr_ctrl_fb,
    lastData_.mr_ctrl_fb,
    mr_ctrl_fb_empty_str,
    [](const auto& msg)
    {
      return std::to_string(msg->position_integral_error.x) + "," + std::to_string(msg->position_integral_error.y) +
             "," + std::to_string(msg->position_integral_error.z) + "," + std::to_string(msg->angle_integral_error.x) +
             "," + std::to_string(msg->angle_integral_error.y) + "," + std::to_string(msg->angle_integral_error.z) +
             ",";
    });
  return csv_line + "\n";
}

FlightLogsWidgetGCS::FlightLogsWidgetGCS() : spinner_(Qt::WindowModal, this)
{
  read_button_ = new QPushButton("Read");
  clean_button_ = new QPushButton("Clean");

  read_button_->setFixedSize(kButtonWidth, kButtonHeight);
  clean_button_->setFixedSize(kButtonWidth, kButtonHeight);

  read_button_->setEnabled(true);
  clean_button_->setEnabled(false);

  log_list_ = new qt::ListWidget();
  log_list_->setSelectionMode(QListWidget::SingleSelection);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(new qt::Label("Ground Station", kPSize1, QFont::Bold));
  cols->addStretch();
  cols->addWidget(read_button_);
  cols->addWidget(clean_button_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(cols);
  rows->addWidget(log_list_);

  setLayout(rows);

  // Connection
  connect(read_button_, &QPushButton::clicked, this, &self::onReadButtonClicked);
  connect(clean_button_, &QPushButton::clicked, this, &self::onCleanButtonClicked);
  connect(log_list_, &QListWidget::itemClicked, this, &self::onListItemClicked);
}

void FlightLogsWidgetGCS::addLog(const QString& log_name)
{
  const auto list_item = new qt::ListWidgetItem();
  list_item->setSizeHint(QSize(0, kListItemHeight));
  list_item->setData(Qt::UserRole, log_name);
  log_list_->addItem(list_item);

  const auto widget = new FlightLogItemWidgetGCS(log_name);
  connect(widget, &FlightLogItemWidgetGCS::exportButtonClicked, this, &self::onExportButtonClicked);
  connect(widget, &FlightLogItemWidgetGCS::deleteButtonClicked, this, &self::onDeleteButtonClicked);
  log_list_->setItemWidget(list_item, widget);
}

void FlightLogsWidgetGCS::removeLog(const QString& log_name)
{
  const auto list_item = findLog(log_name);
  TOBAS_CHECK(list_item);
  log_list_->remove(list_item);
}

QListWidgetItem* FlightLogsWidgetGCS::findLog(const QString& log_name)
{
  for (int row = 0; row < log_list_->count(); ++row) {
    const auto list_item = log_list_->item(row);
    const auto log_widget = qt::qConstPointerCast<FlightLogItemWidgetGCS>(log_list_->itemWidget(list_item));

    if (log_widget->logName() == log_name) {
      return list_item;
    }
  }

  return nullptr;
}

void FlightLogsWidgetGCS::clearLogs()
{
  log_list_->clear();
}

void FlightLogsWidgetGCS::sortLogs()
{
  log_list_->sortItems();
}

QString FlightLogsWidgetGCS::currentLogName() const
{
  const auto cur_item = log_list_->currentItem();
  if (!cur_item) {
    qWarning() << "Log name not selected.";
    return "";
  }

  return cur_item->data(Qt::UserRole).toString();
}

void FlightLogsWidgetGCS::setCurrentLogName(const QString& log_name)
{
  for (int i = 0; i < log_list_->count(); ++i) {
    const auto item = log_list_->item(i);
    if (item->data(Qt::UserRole).toString() == log_name) {
      log_list_->setCurrentItem(item);
      return;
    }
  }

  qWarning() << log_name << " not found.";
}

void FlightLogsWidgetGCS::onReadButtonClicked()
{
  // 現在選択されているアイテムを取得
  const auto cur_text = currentLogName();

  clearLogs();

  const auto rosbag_dir = ros2::expandUser(tobas::kRosbagDirHome);
  if (!fs::is_directory(rosbag_dir)) {
    fs::create_directories(rosbag_dir);
  }

  try {
    for (const auto& entry : fs::directory_iterator(rosbag_dir)) {
      const QString log_name(entry.path().filename().c_str());
      addLog(log_name);
    }
  }
  catch (const std::exception& e) {
    qt::qErrorBox(this, "Exception occurred while iterating " + QString::fromStdString(rosbag_dir) + ": " + e.what());
    return;
  }

  if (log_list_->count() == 0) {
    qt::qWarnBox(this, "There are no flight logs saved on the ground control station.");
    return;
  }

  sortLogs();

  // 選択されていたアイテムを再び選択
  if (!cur_text.isEmpty()) {
    setCurrentLogName(cur_text);
  }

  clean_button_->setEnabled(true);
}

void FlightLogsWidgetGCS::onCleanButtonClicked()
{
  if (!qt::yesOrNo(this, "Do you want to clean all the flight logs saved in the GCS?", qt::WARN)) {
    return;
  }

  const auto rosbag_dir = ros2::expandUser(tobas::kRosbagDirHome);
  if (!fs::is_directory(rosbag_dir)) {
    fs::create_directories(rosbag_dir);
  }

  try {
    for (const auto& entry : fs::directory_iterator(rosbag_dir)) {
      if (fs::remove_all(entry.path()) == 0) {
        qt::qErrorBox(this, "Failed to delete " + QString::fromStdString(entry.path()));
        return;
      }
    }
  }
  catch (const std::exception& e) {
    qt::qErrorBox(
      this, "Exception occurred while iterating " + QString::fromStdString(rosbag_dir) + ": " + QString(e.what()));
    return;
  }

  if (log_list_->currentItem()) {
    log_list_->deselect();
    Q_EMIT logDeselected();
  }

  clearLogs();
}

void FlightLogsWidgetGCS::onExportFinished()
{
  spinner_.stop();
  qt::qInfoBox(this, "Flight log has been exported successfully.");
}

void FlightLogsWidgetGCS::onExportError(const QString& err)
{
  spinner_.stop();
  qt::qErrorBox(this, "Export Error" + err);
}

void FlightLogsWidgetGCS::onExportButtonClicked(const QString& log_name)
{
  fs::path defaultCsvName(log_name.toStdString());
  defaultCsvName.replace_extension(".csv");

  QString savePath = QFileDialog::getSaveFileName(
    this, "Select the directory", QString::fromStdString(defaultCsvName.string()), "CSV Files (*.csv)");

  if (savePath.isEmpty()) {
    return;
  }

  spinner_.start();
  QThread* thread = new QThread;
  CsvExportWorker* worker = new CsvExportWorker();

  connect(thread, &QThread::started, worker, [worker, log_name, savePath]() { worker->process(log_name, savePath); });

  // 処理完了時の後始末
  connect(worker, &CsvExportWorker::finished, thread, &QThread::quit);
  connect(worker, &CsvExportWorker::finished, worker, &CsvExportWorker::deleteLater);
  connect(thread, &QThread::finished, thread, &QThread::deleteLater);

  // GUIへの通知接続
  connect(worker, &CsvExportWorker::finished, this, &FlightLogsWidgetGCS::onExportFinished);
  connect(worker, &CsvExportWorker::error, this, &FlightLogsWidgetGCS::onExportError);

  // 4. スレッド開始
  thread->start();
}

void FlightLogsWidgetGCS::onDeleteButtonClicked(const QString& log_name)
{
  const auto log_path = ros2::expandUser(tobas::kRosbagDirHome) / log_name.toStdString();

  if (!qt::yesOrNo(this, "Do you want to delete flight log \"" + log_name + "\"?", qt::WARN)) {
    return;
  }

  if (fs::remove_all(log_path) == 0) {
    qt::qErrorBox(this, "Failed to delete " + QString::fromStdString(log_path));
    return;
  }

  if (currentLogName() == log_name) {
    log_list_->deselect();
    Q_EMIT logDeselected();
  }

  removeLog(log_name);
}

void FlightLogsWidgetGCS::onListItemClicked(QListWidgetItem* item)
{
  const auto log_widget = qt::qConstPointerCast<FlightLogItemWidgetGCS>(log_list_->itemWidget(item));
  Q_EMIT logSelected(log_widget->logName());
}
}  // namespace log
}  // namespace gui
