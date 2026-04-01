// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/msgs/double.pb.h>
#include <gz/gui/Plugin.hh>
#include <gz/rendering/Scene.hh>
#include <gz/transport/Node.hh>

namespace tobas
{
namespace gazebo
{
class TobasRenderInfo : public gz::gui::Plugin
{
  Q_OBJECT

  using self = TobasRenderInfo;
  using super = gz::gui::Plugin;

  static constexpr double kAlpha = 0.05;
  static constexpr auto kPublishPeriod = std::chrono::milliseconds(1000);

public:
  explicit TobasRenderInfo();

  void LoadConfig(const tinyxml2::XMLElement* elem) override;

private:
  bool eventFilter(QObject* obj, QEvent* event) override;

  void onRender();

  gz::transport::Node node_;
  gz::transport::Node::Publisher fps_pub_;

  gz::rendering::ScenePtr scene_;

  std::chrono::steady_clock::time_point t_last_tick_;
  std::chrono::steady_clock::time_point t_last_pub_;

  double fps_ema_ = NAN;
  gz::msgs::Double fps_msg_;
};
}  // namespace gazebo
}  // namespace tobas
