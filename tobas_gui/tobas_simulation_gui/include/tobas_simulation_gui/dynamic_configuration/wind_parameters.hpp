// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/slider_text.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

#include <tobas_gazebo_msgs/srv/get_wind_params.hpp>
#include <tobas_gazebo_msgs/srv/set_wind_params.hpp>

namespace tobas
{
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

  void updateNamespace(const std::string& ns);

  bool start();
  void reset();

private:
  const rclcpp::Node::SharedPtr node_;
  ros2::SyncServiceClient<GetSrv>::SharedPtr get_sc_;
  ros2::SyncServiceClient<SetSrv>::SharedPtr set_sc_;

  QPushButton* reset_button_;

  qt::DoubleSliderTextWidget* mean_speed_;
  qt::IntSliderTextWidget* direction_;
  qt::DoubleSliderTextWidget* gust_speed_factor_;
  qt::DoubleSliderTextWidget* gust_duration_;
  qt::DoubleSliderTextWidget* gust_interval_;

  double init_mean_speed_;
  int init_direction_;
  double init_gust_speed_factor_;
  double init_gust_duration_;
  double init_gust_interval_;

  double getMeanSpeed() const;        // [m/s]
  double getDirection() const;        // [rad]
  double getGustSpeedFactor() const;  // [-]
  double getGustDuration() const;     // [s]
  double getGustInterval() const;     // [s]

  void setMeanSpeed(double value);
  void setDirection(double value_rad);
  void setGustSpeedFactor(double value);
  void setGustDuration(double value);
  void setGustInterval(double value);

  bool loadSimParams();
  bool sendGuiParams();

private Q_SLOTS:
  void onResetButtonClicked();
  void onValueChanged();
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
