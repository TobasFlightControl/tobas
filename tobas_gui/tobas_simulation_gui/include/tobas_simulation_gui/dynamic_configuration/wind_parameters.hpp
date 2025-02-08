#pragma once

#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_qt_tools/widgets/slider_text.hpp>
#include <tobas_gazebo_msgs/srv/get_wind_params.hpp>
#include <tobas_gazebo_msgs/srv/set_wind_params.hpp>

namespace gui
{
namespace sim
{
class WindParamsWidget : public QWidget
{
  Q_OBJECT

  using self = WindParamsWidget;
  using super = QWidget;
  using GetSrv = tobas_gazebo_msgs::srv::GetWindParams;
  using SetSrv = tobas_gazebo_msgs::srv::SetWindParams;

public:
  explicit WindParamsWidget(rclcpp::Node::SharedPtr node);

  bool start(const std::string& ns);
  void terminate();

private:
  const rclcpp::Node::SharedPtr node_;
  ros2::SyncServiceClient<GetSrv>::SharedPtr get_sc_;
  ros2::SyncServiceClient<SetSrv>::SharedPtr set_sc_;

  qt::DoubleSliderTextWidget* mean_speed_;
  qt::DoubleSliderTextWidget* direction_;
  qt::DoubleSliderTextWidget* gust_speed_factor_;
  qt::DoubleSliderTextWidget* gust_duration_;
  qt::DoubleSliderTextWidget* gust_interval_;

  void reset();
  void loadCurrentParams();

private Q_SLOTS:
  void onValueChanged();
};
}  // namespace sim
}  // namespace gui
