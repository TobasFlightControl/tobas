#pragma once

#include <functional>

#include <QObject>
#include <QPushButton>
#include <QThread>

#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

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
#include <tobas_msgs/msg/odometry.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/vibration_level.hpp>

#include "tobas_flight_log_gui/log_viewer/message_decoder.hpp"

namespace gui
{
namespace log
{

class CsvExportWorker : public QObject
{
  Q_OBJECT

  std::string exportCsvHeader;

public:
  explicit CsvExportWorker(QObject* parent = nullptr) : QObject(parent)
  {
  }

  struct CurrentData
  {
    std::shared_ptr<tobas_msgs::msg::Odometry> odom;
    std::shared_ptr<tobas_msgs::msg::Imu> imu;
    std::shared_ptr<tobas_msgs::msg::MagneticField> mag;
    std::shared_ptr<tobas_msgs::msg::Gnss> gnss;
    std::shared_ptr<tobas_msgs::msg::RCInput> rcin;
    std::shared_ptr<tobas_msgs::msg::Battery> battery;
    std::shared_ptr<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd;
    std::shared_ptr<tobas_msgs::msg::Cpu> cpu;
    std::shared_ptr<tobas_msgs::msg::RotorStateArray> rotor_states;
    std::shared_ptr<tobas_msgs::msg::RotorSpeedArray> rotor_speeds;
    std::shared_ptr<tobas_msgs::msg::Latency> latency;
    std::shared_ptr<tobas_msgs::msg::VibrationLevel> vibration_level;
    std::shared_ptr<tobas_kdl_msgs::msg::WrenchStamped> disturbance_force;
    std::shared_ptr<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb;
    std::shared_ptr<tobas_debug_msgs::msg::MulticopterControllerFeedback> mr_ctrl_fb;
  } curData_;
  struct LastData
  {
    std::shared_ptr<tobas_msgs::msg::Odometry> odom = nullptr;
    std::shared_ptr<tobas_msgs::msg::Imu> imu = nullptr;
    std::shared_ptr<tobas_msgs::msg::MagneticField> mag = nullptr;
    std::shared_ptr<tobas_msgs::msg::Gnss> gnss = nullptr;
    std::shared_ptr<tobas_msgs::msg::RCInput> rcin = nullptr;
    std::shared_ptr<tobas_msgs::msg::Battery> battery = nullptr;
    std::shared_ptr<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd = nullptr;
    std::shared_ptr<tobas_msgs::msg::Cpu> cpu = nullptr;
    std::shared_ptr<tobas_msgs::msg::RotorStateArray> rotor_states = nullptr;
    std::shared_ptr<tobas_msgs::msg::RotorSpeedArray> rotor_speeds = nullptr;
    std::shared_ptr<tobas_msgs::msg::Latency> latency = nullptr;
    std::shared_ptr<tobas_msgs::msg::VibrationLevel> vibration_level = nullptr;
    std::shared_ptr<tobas_kdl_msgs::msg::WrenchStamped> disturbance_force = nullptr;
    std::shared_ptr<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb = nullptr;
    std::shared_ptr<tobas_debug_msgs::msg::MulticopterControllerFeedback> mr_ctrl_fb = nullptr;

  } lastData_;

  bool open(const std::string& rosbag_path);
  bool reindex(const std::string& rosbag_path);
  template <typename T, typename Func>
  std::string getLogString(
    const std::shared_ptr<T>& cur_ptr,
    std::shared_ptr<T>& last_ptr,
    const std::string& empty_str,
    Func formatter);
  std::string makeCsvRow(const auto& cur_time);

private:
  size_t num_rotors_;

  rosbag2_cpp::Reader reader_;
  MessageDecoder<tobas_msgs::msg::Odometry> odom_decoder_;
  MessageDecoder<tobas_msgs::msg::Imu> imu_decoder_;
  MessageDecoder<tobas_msgs::msg::MagneticField> mag_decoder_;
  MessageDecoder<tobas_msgs::msg::Gnss> gnss_decoder_;
  MessageDecoder<tobas_msgs::msg::RCInput> rcin_decoder_;
  MessageDecoder<tobas_msgs::msg::Battery> battery_decoder_;
  MessageDecoder<tobas_msgs::msg::Cpu> cpu_decoder_;
  MessageDecoder<tobas_msgs::msg::RotorStateArray> rotor_states_decoder_;
  MessageDecoder<tobas_msgs::msg::RotorSpeedArray> rotor_speeds_decoder_;
  // MessageDecoder<tobas_msgs::msg::JointStateArray> joint_states_decoder_;
  // MessageDecoder<tobas_msgs::msg::JointCommandArray> joint_commands_decoder_;
  MessageDecoder<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_decoder_;
  MessageDecoder<tobas_msgs::msg::Latency> latency_decoder_;
  MessageDecoder<tobas_msgs::msg::VibrationLevel> vibe_decoder_;
  MessageDecoder<tobas_kdl_msgs::msg::WrenchStamped> wrench_decoder_;
  MessageDecoder<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb_decoder_;
  MessageDecoder<tobas_debug_msgs::msg::MulticopterControllerFeedback> mr_ctrl_fb_decoder_;

public Q_SLOTS:
  void process(const QString& log_name, const QString& savePath);

Q_SIGNALS:
  void finished();
  void error(const QString& err);
};

class FlightLogsWidgetGCS : public QWidget
{
  Q_OBJECT

  using self = FlightLogsWidgetGCS;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr int kListItemHeight = 40;

Q_SIGNALS:
  void logSelected(const QString& log_name);
  void logDeselected();

public:
  explicit FlightLogsWidgetGCS();

  void addLog(const QString& log_name);
  void removeLog(const QString& log_name);
  QListWidgetItem* findLog(const QString& log_name);

  void clearLogs();
  void sortLogs();

private:
  QPushButton* read_button_;
  QPushButton* clean_button_;

  qt::ListWidget* log_list_;

  qt::WaitSpinnerWidget spinner_;

  QString currentLogName() const;
  void setCurrentLogName(const QString& log_name);

private Q_SLOTS:
  void onReadButtonClicked();
  void onCleanButtonClicked();
  void onExportButtonClicked(const QString& log_name);
  void onExportFinished();
  void onExportError(const QString& err);
  void onDeleteButtonClicked(const QString& log_name);
  void onListItemClicked(QListWidgetItem* item);
};
}  // namespace log
}  // namespace gui
