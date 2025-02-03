#pragma once

#include <filesystem>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "./plot_tab.hpp"
#include "./playback_control.hpp"

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

  std::array<PlotTabWidget*, 2> plot_tabs_;
  PlaybackControlWidget* playback_ctrl_;

  tobas_msgs::msg::Odometry odom_;
  tobas_msgs::msg::ImuWithCovarianceStamped imu_;
  tobas_msgs::msg::MagneticFieldWithCovarianceStamped mag_;
  tobas_msgs::msg::Gps gps_;
  tobas_msgs::msg::Battery batt_;
  tobas_msgs::msg::Latency latency_;
  tobas_kdl_msgs::msg::WrenchStamped dist_force_;

  rclcpp::Serialization<tobas_msgs::msg::Odometry> odom_ser_;
  rclcpp::Serialization<tobas_msgs::msg::ImuWithCovarianceStamped> imu_ser_;
  rclcpp::Serialization<tobas_msgs::msg::MagneticFieldWithCovarianceStamped> mag_ser_;
  rclcpp::Serialization<tobas_msgs::msg::Gps> gps_ser_;
  rclcpp::Serialization<tobas_msgs::msg::Battery> batt_ser_;
  rclcpp::Serialization<tobas_msgs::msg::Latency> latency_ser_;
  rclcpp::Serialization<tobas_kdl_msgs::msg::WrenchStamped> dist_force_ser_;

  void reset();
  void setPlotData(double time_from_start);

private Q_SLOTS:
  void onPlaybackTimeChanged(double time_from_start);
};
}  // namespace log
}  // namespace gui
