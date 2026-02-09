#pragma once

#include <QThread>
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

#include "../message_decoder.hpp"

namespace gui
{
namespace log
{
class CsvExportThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit CsvExportThread(const QString& log_name, const QString& save_path);

  void run() override;

private:
  const QString log_name_;
  const QString save_path_;

  struct Data
  {
    tobas_msgs::msg::Odometry::SharedPtr odom;
    tobas_msgs::msg::Imu::SharedPtr imu;
    tobas_msgs::msg::MagneticField::SharedPtr mag;
    tobas_msgs::msg::Gnss::SharedPtr gnss;
    tobas_msgs::msg::RCInput::SharedPtr rcin;
    tobas_msgs::msg::Battery::SharedPtr battery;
    tobas_msgs::msg::IcePropulsionSystemCommand::SharedPtr ice_cmd;
    tobas_msgs::msg::Cpu::SharedPtr cpu;
    tobas_msgs::msg::RotorStateArray::SharedPtr rotor_states;
    tobas_msgs::msg::RotorSpeedArray::SharedPtr rotor_speeds;
    tobas_msgs::msg::Latency::SharedPtr latency;
    tobas_msgs::msg::VibrationLevel::SharedPtr vibration_level;
    tobas_kdl_msgs::msg::WrenchStamped::SharedPtr disturbance_force;
    tobas_debug_msgs::msg::ObserverFeedback::SharedPtr obsv_fb;
    tobas_debug_msgs::msg::MulticopterControllerFeedback::SharedPtr mr_ctrl_fb;
  } cur_data_, last_data_;

  std::vector<std::string> rotor_link_names_;

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
  MessageDecoder<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_decoder_;
  MessageDecoder<tobas_msgs::msg::Latency> latency_decoder_;
  MessageDecoder<tobas_msgs::msg::VibrationLevel> vibe_decoder_;
  MessageDecoder<tobas_kdl_msgs::msg::WrenchStamped> wrench_decoder_;
  MessageDecoder<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb_decoder_;
  MessageDecoder<tobas_debug_msgs::msg::MulticopterControllerFeedback> mr_ctrl_fb_decoder_;

  bool openRosBag(const std::string& path);

  bool rotorLinkNamesValid(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg);
  bool rotorLinkNamesValid(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& msg);

  std::string makeCsvHeader() const;
  std::string makeCsvRow(const rcutils_time_point_value_t& cur_time);
};
}  // namespace log
}  // namespace gui
