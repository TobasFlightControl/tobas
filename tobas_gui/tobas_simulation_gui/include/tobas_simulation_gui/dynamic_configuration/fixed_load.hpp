// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_qt_tools/widgets/vector3d_edit_vertical.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

#include <tobas_gazebo_msgs/srv/attach_fixed_load.hpp>
#include <tobas_gazebo_msgs/srv/detach_fixed_load.hpp>

namespace tobas
{
namespace gui
{
namespace sim
{
class FixedLoadWidget : public QWidget
{
  Q_OBJECT

  using self = FixedLoadWidget;
  using AttachSrv = tobas_gazebo_msgs::srv::AttachFixedLoad;
  using DetachSrv = tobas_gazebo_msgs::srv::DetachFixedLoad;

public:
  explicit FixedLoadWidget();

  void reset();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);
  void clearRosInterfaces();

private:
  std::optional<ros2::SyncServiceClient<AttachSrv>> attach_sc_;
  std::optional<ros2::SyncServiceClient<DetachSrv>> detach_sc_;

  qt::ToggleButton* attach_detach_btn_;

  qt::Vector3dEditVertical* load_position_;
  qt::Vector3dEditVertical* load_angle_;
  qt::Vector3dEditVertical* load_size_;
  qt::DoubleSpinBox* load_mass_;

  void setParamsToDefault();

private Q_SLOTS:
  void onAttachButtonClicked();
  void onDetachButtonClicked();
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
