// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QButtonGroup>
#include <rclcpp/node.hpp>

#include <tobas_qt_tools/widgets/stacked_widget.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "../base.hpp"
#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sc
{
class MagCalibrationWidget : public BaseWidget
{
  Q_OBJECT

public:
  explicit MagCalibrationWidget(const rqt::RosQtBridge& _bridge);

  const char* title() const override;
  void reset() override;
  void updateInternalDataStructures() override;
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns) override;
  void clearRosInterfaces() override;

private:
  QButtonGroup* btn_group_;
  qt::StackedWidget* stack_;

  void addMagCalibWidget(BaseMagCalibWidget* _widget, const QString& _label, int _id);

  BaseMagCalibWidget* getWidget(int _index);

  void setCurrentIndex(int _index);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
