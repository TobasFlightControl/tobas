// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QHBoxLayout>

#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "./speedmeter.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
class RotorsViewerWiddget : public qt::ScrollArea
{
  Q_OBJECT

  using self = RotorsViewerWiddget;
  using super = qt::ScrollArea;

  static constexpr char kAliveBackgroundColor[] = "transparent";
  static constexpr char kDeadBackgroundColor[] = "red";

public:
  explicit RotorsViewerWiddget(const RosQtBridge& bridge, const Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const Drone& drone_;

  std::map<std::string, SpeedmeterWidget*> meters_;
  QHBoxLayout* cols_;

  void setSpeed(const std::string& link_name, const double& rps);

  static QString bottomText(int rpm);

private Q_SLOTS:
  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& msg);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
