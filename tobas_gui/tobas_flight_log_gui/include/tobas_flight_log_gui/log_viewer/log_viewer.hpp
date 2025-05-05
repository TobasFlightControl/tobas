#pragma once

#include <filesystem>

#include <rosbag2_cpp/reader.hpp>

#include "./message_decoder.hpp"
#include "./playback_control.hpp"
#include "./plot_tab.hpp"

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

  void setLogName(const QString& log_name);

private:
  std::filesystem::path log_path_;
  rosbag2_cpp::Reader reader_;
  std::unordered_set<std::string> decode_fail_topics_;

  MessageDecoder<tobas_msgs::msg::Odometry> odom_decoder_;
  MessageDecoder<tobas_msgs::msg::ImuWithCovarianceStamped> imu_decoder_;
  MessageDecoder<tobas_msgs::msg::MagneticFieldWithCovarianceStamped> mag_decoder_;
  MessageDecoder<tobas_msgs::msg::Gnss> gnss_decoder_;
  MessageDecoder<tobas_msgs::msg::Battery> battery_decoder_;
  MessageDecoder<tobas_msgs::msg::RotorStateArray> cur_rotor_states_decoder_;
  MessageDecoder<tobas_msgs::msg::RotorSpeedArray> tar_rotor_speeds_decoder_;
  MessageDecoder<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_decoder_;
  MessageDecoder<tobas_msgs::msg::Latency> sampling_time_decoder_;
  MessageDecoder<tobas_msgs::msg::Latency> ctrl_latency_decoder_;
  MessageDecoder<tobas_kdl_msgs::msg::WrenchStamped> dist_force_decoder_;
  MessageDecoder<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb_decoder_;
  MessageDecoder<tobas_debug_msgs::msg::MultiRotorControllerFeedback> mr_ctrl_fb_decoder_;

  std::array<PlotTabWidget*, 2> plot_tabs_;
  PlaybackControlWidget* playback_ctrl_;

  void reset();
  void setPlotData(double time_from_start);

private Q_SLOTS:
  void onPlaybackTimeChanged(double time_from_start);
};
}  // namespace log
}  // namespace gui
