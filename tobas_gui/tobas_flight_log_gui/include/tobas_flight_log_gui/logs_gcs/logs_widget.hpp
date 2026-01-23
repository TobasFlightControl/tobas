#pragma once

#include <functional>

#include <QObject>
#include <QPushButton>
#include <QThread>

#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

#include <rosbag2_cpp/reader.hpp>

#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/gnss.hpp>
#include <tobas_msgs/msg/imu.hpp>
#include <tobas_msgs/msg/magnetic_field.hpp>
#include <tobas_msgs/msg/odometry.hpp>

#include "tobas_flight_log_gui/log_viewer/message_decoder.hpp"

namespace gui
{
namespace log
{

// class LogExportWorker : public QObject
// {
//   Q_OBJECT

// public:
//   explicit LogExportWorker(const QString& log_name) : log_name_(log_name) {}
//   struct CurrentData
//   {
//     std::shared_ptr<tobas_msgs::msg::Odometry> cur_odom;
//     tobas_msgs::msg::Imu cur_imu;
//     std::shared_ptr<tobas_msgs::msg::MagneticField> mag_data;
//     std::shared_ptr<tobas_msgs::msg::Battery> cur_battery;
//     std::shared_ptr<tobas_msgs::msg::Gnss> cur_gnss;
//   } curData_;
//   struct LastData
//   {
//     std::shared_ptr<tobas_msgs::msg::Odometry> last_odom = nullptr;
//     std::shared_ptr<tobas_msgs::msg::Battery> last_battery = nullptr;
//     std::shared_ptr<tobas_msgs::msg::Gnss> last_gnss = nullptr;
//   }lastData_;

// public Q_SLOTS:
//   void process();

// Q_SIGNALS:
//   void finished();
//   void error(QString err);

// private:
//   QString log_name_;

//   std::string makeCsvRow(const auto& cur_time);

//   template <typename T, typename Func>
//   std::string getLogString(const std::shared_ptr<T>& cur_ptr,
//                          std::shared_ptr<T>& last_ptr,
//                          const std::string& empty_str,
//                          Func formatter);
//   std::string makeCsvRow(const auto& cur_time);
// };
class FlightLogsWidgetGCS : public QWidget
{
  Q_OBJECT

  using self = FlightLogsWidgetGCS;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr int kListItemHeight = 40;

  std::string exportCsvHeader = "time,\
    imu_raw/accel/x, imu_raw/accel/y, imu_raw/accel/z,\
    imu_raw/gyro/x, imu_raw/gyro/y, imu_raw/gyro/z,\
    imu_raw/dgyro/x, imu_raw/dgyro/y, imu_raw/dgyro/z,\
    gnss/latitude, gnss/longitude, gnss/altitude,\
    gnss/EastSpeed, gnss/NorthSpeed, gnss/UpSpeed,\
    battery/voltage, battery/current\n";

Q_SIGNALS:
  void logSelected(const QString& log_name);
  void logDeselected();

public:
  explicit FlightLogsWidgetGCS();

  struct CurrentData
  {
    std::shared_ptr<tobas_msgs::msg::Odometry> cur_odom;
    tobas_msgs::msg::Imu cur_imu;
    std::shared_ptr<tobas_msgs::msg::MagneticField> mag_data;
    std::shared_ptr<tobas_msgs::msg::Battery> cur_battery;
    std::shared_ptr<tobas_msgs::msg::Gnss> cur_gnss;
  } curData_;
  struct LastData
  {
    std::shared_ptr<tobas_msgs::msg::Odometry> last_odom = nullptr;
    std::shared_ptr<tobas_msgs::msg::Battery> last_battery = nullptr;
    std::shared_ptr<tobas_msgs::msg::Gnss> last_gnss = nullptr;
  } lastData_;

  template <typename T, typename Func>
  std::string getLogString(
    const std::shared_ptr<T>& cur_ptr,
    std::shared_ptr<T>& last_ptr,
    const std::string& empty_str,
    Func formatter);
  std::string makeCsvRow(const auto& cur_time);
  void addLog(const QString& log_name);
  void removeLog(const QString& log_name);
  bool open(const std::string& rosbag_path);
  bool reindex(const std::string& rosbag_path);
  QListWidgetItem* findLog(const QString& log_name);

  void clearLogs();
  void sortLogs();

private:
  rosbag2_cpp::Reader reader_;

  QPushButton* read_button_;
  QPushButton* clean_button_;

  qt::ListWidget* log_list_;

  qt::WaitSpinnerWidget spinner_;

  QString currentLogName() const;
  void setCurrentLogName(const QString& log_name);

  MessageDecoder<tobas_msgs::msg::Odometry> odom_decoder_;
  MessageDecoder<tobas_msgs::msg::Imu> imu_decoder_;
  MessageDecoder<tobas_msgs::msg::MagneticField> mag_decoder_;
  MessageDecoder<tobas_msgs::msg::Gnss> gnss_decoder_;
  // MessageDecoder<tobas_msgs::msg::RCInput> rcin_decoder_;
  MessageDecoder<tobas_msgs::msg::Battery> battery_decoder_;
  // MessageDecoder<tobas_msgs::msg::Cpu> cpu_decoder_;
  // MessageDecoder<tobas_msgs::msg::RotorStateArray> rotor_states_decoder_;
  // MessageDecoder<tobas_msgs::msg::RotorSpeedArray> rotor_speeds_decoder_;
  // MessageDecoder<tobas_msgs::msg::JointStateArray> joint_states_decoder_;
  // MessageDecoder<tobas_msgs::msg::JointCommandArray> joint_commands_decoder_;
  // MessageDecoder<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_decoder_;
  // MessageDecoder<tobas_msgs::msg::Latency> latency_decoder_;
  // MessageDecoder<tobas_msgs::msg::VibrationLevel> vibe_decoder_;
  // MessageDecoder<tobas_kdl_msgs::msg::WrenchStamped> wrench_decoder_;
  // MessageDecoder<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb_decoder_;
  // MessageDecoder<tobas_debug_msgs::msg::MulticopterControllerFeedback> mr_ctrl_fb_decoder_;

private Q_SLOTS:
  void onReadButtonClicked();
  void onCleanButtonClicked();
  void onExportButtonClicked(const QString& log_name);
  // void onExportFinished();
  void onDeleteButtonClicked(const QString& log_name);
  void onListItemClicked(QListWidgetItem* item);
};
}  // namespace log
}  // namespace gui
