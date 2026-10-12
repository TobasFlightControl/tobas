// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QHBoxLayout>

#include <tobas_drone_core/drone.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "./speedmeter.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
class RotorsViewerWiddget : public QWidget
{
  Q_OBJECT

  using self = RotorsViewerWiddget;

public:
  explicit RotorsViewerWiddget(const rqt::RosQtBridge& _bridge, const Drone& _drone);

  void reset();
  void updateInternalDataStructures();

private:
  const Drone& drone_;

  std::map<std::string, SpeedmeterWidget*> meters_;
  QHBoxLayout* cols_;

  void setSpeed(const std::string& _link_name, double _rps);

  static QString bottomText(int _rpm);

private Q_SLOTS:
  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& _msg);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& _msg);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
