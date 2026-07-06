// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rviz_common/display.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

namespace rviz_common
{
class VisualizationFrame;
class VisualizationManager;
class DisplayGroup;
}  // namespace rviz_common

namespace tobas
{
namespace rviz
{
class RvizFrameManager
{
public:
  explicit RvizFrameManager(int argc, char** argv, const std::string& node_name);
  explicit RvizFrameManager(const std::string& node_name);

  void initialize(const QString& config_path, QWidget* parent = nullptr);

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rvizNode();
  rclcpp::Node::SharedPtr rawNode();

  QWidget* widget();

  QString getFixedFrame() const;
  void setFixedFrame(const QString& frame);
  void setOrbitView(float distance, float yaw, float pitch, float focal_x, float focal_y, float focal_z);
  void resetTime();

  std::vector<rviz_common::Display*> getDisplays(const QString& name);

private:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> node_;

  rviz_common::VisualizationFrame* frame_;
  rviz_common::VisualizationManager* manager_;
  rviz_common::DisplayGroup* display_group_;

  void removeDefaultColorMaterials();
};
}  // namespace rviz
}  // namespace tobas
