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
  std::filesystem::path log_path_;
  std::unordered_set<std::string> decode_fail_topics_;
  rosbag2_cpp::Reader reader_;

  QVector<tobas_msgs::msg::OdometryWithCovarianceStamped> odom_data_;
  QVector<tobas_msgs::msg::OdometryStamped> setpoint_data_;
  QVector<tobas_msgs::msg::Imu> raw_imu_data_;
  QVector<tobas_msgs::msg::Imu> filt_imu_data_;
  QVector<tobas_msgs::msg::MagneticField> mag_data_;
  QVector<tobas_msgs::msg::Gnss> gnss_data_;
  QVector<tobas_msgs::msg::RCInput> rcin_data_;
  QVector<tobas_msgs::msg::Battery> battery_data_;
  QVector<tobas_msgs::msg::Cpu> cpu_data_;
  QVector<tobas_msgs::msg::RotorStateArray> cur_rotor_states_data_;
  QVector<tobas_msgs::msg::RotorSpeedArray> tar_rotor_speeds_data_;
  QVector<tobas_msgs::msg::JointStateArray> cur_joint_states_data_;
  QVector<tobas_msgs::msg::JointCommandArray> tar_joint_positions_data_;
  QVector<tobas_msgs::msg::JointCommandArray> tar_joint_velocities_data_;
  QVector<tobas_msgs::msg::JointCommandArray> tar_joint_efforts_data_;
  QVector<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_data_;
  QVector<tobas_msgs::msg::Latency> sampling_time_data_;
  QVector<tobas_msgs::msg::Latency> ctrl_latency_data_;
  QVector<tobas_msgs::msg::VibrationLevel> vibe_data_;
  QVector<tobas_kdl_msgs::msg::WrenchStamped> dist_force_data_;
  QVector<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb_data_;
  QVector<tobas_debug_msgs::msg::MulticopterControllerFeedback> mr_ctrl_fb_data_;

  MessageDecoderCache<tobas_msgs::msg::OdometryWithCovarianceStamped> odom_cov_decoder_;
  MessageDecoderCache<tobas_msgs::msg::OdometryStamped> odom_decoder_;
  MessageDecoderCache<tobas_msgs::msg::Imu> imu_decoder_;
  MessageDecoderCache<tobas_msgs::msg::MagneticField> mag_decoder_;
  MessageDecoderCache<tobas_msgs::msg::Gnss> gnss_decoder_;
  MessageDecoderCache<tobas_msgs::msg::RCInput> rcin_decoder_;
  MessageDecoderCache<tobas_msgs::msg::Battery> battery_decoder_;
  MessageDecoderCache<tobas_msgs::msg::Cpu> cpu_decoder_;
  MessageDecoderCache<tobas_msgs::msg::RotorStateArray> rotor_states_decoder_;
  MessageDecoderCache<tobas_msgs::msg::RotorSpeedArray> rotor_speeds_decoder_;
  MessageDecoderCache<tobas_msgs::msg::JointStateArray> joint_states_decoder_;
  MessageDecoderCache<tobas_msgs::msg::JointCommandArray> joint_commands_decoder_;
  MessageDecoderCache<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_decoder_;
  MessageDecoderCache<tobas_msgs::msg::Latency> latency_decoder_;
  MessageDecoderCache<tobas_msgs::msg::VibrationLevel> vibe_decoder_;
  MessageDecoderCache<tobas_kdl_msgs::msg::WrenchStamped> wrench_decoder_;
  MessageDecoderCache<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb_decoder_;
  MessageDecoderCache<tobas_debug_msgs::msg::MulticopterControllerFeedback> mr_ctrl_fb_decoder_;

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
