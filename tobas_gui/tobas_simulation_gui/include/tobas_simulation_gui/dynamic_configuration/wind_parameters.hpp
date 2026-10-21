// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <QPushButton>

#include <tobas_qt_tools/widgets/slider_text.hpp>
#include <tobas_rqt_tools/sync_service_client.hpp>

#include <tobas_gazebo_msgs/srv/get_wind_params.hpp>
#include <tobas_gazebo_msgs/srv/set_wind_params.hpp>

namespace tobas::gui::sim
{
class WindParamsWidget : public QWidget
{
  Q_OBJECT

  using self = WindParamsWidget;
  using GetSrv = tobas_gazebo_msgs::srv::GetWindParams;
  using SetSrv = tobas_gazebo_msgs::srv::SetWindParams;

public:
  explicit WindParamsWidget();

  void reset();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);
  void clearRosInterfaces();

private:
  std::optional<rqt::SyncServiceClientWrapper<GetSrv>> get_sc_;
  std::optional<rqt::SyncServiceClientWrapper<SetSrv>> set_sc_;

  QPushButton* reset_button_;

  qt::DoubleSliderTextWidget* mean_speed_;
  qt::IntSliderTextWidget* direction_;
  qt::DoubleSliderTextWidget* gust_speed_factor_;
  qt::DoubleSliderTextWidget* gust_duration_;
  qt::DoubleSliderTextWidget* gust_interval_;

  double getMeanSpeed() const;        // [m/s]
  double getDirection() const;        // [rad]
  double getGustSpeedFactor() const;  // [-]
  double getGustDuration() const;     // [s]
  double getGustInterval() const;     // [s]

  void setMeanSpeed(double _value);
  void setDirection(double _value_rad);
  void setGustSpeedFactor(double _value);
  void setGustDuration(double _value);
  void setGustInterval(double _value);

  void setParamsToDefault();
  bool sendGuiParams();

private Q_SLOTS:
  void onResetButtonClicked();
  void onValueChanged();
};
}  // namespace tobas::gui::sim
