// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/controller/y_axis_tilt_vtol.hpp"

#include <QVBoxLayout>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace ctrl
{
YAxisTiltVtolWidget::YAxisTiltVtolWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  rows->addStretch();
}

void YAxisTiltVtolWidget::setToDefaults()
{
}

FrameType YAxisTiltVtolWidget::frameType() const
{
  return FrameType::kYAxisTiltVtol;
}

QString YAxisTiltVtolWidget::controllerPackage() const
{
  return "tobas_y_axis_tilt_vtol_controller";
}

QString YAxisTiltVtolWidget::pluginName() const
{
  return "tobas::y_axis_tilt_vtol::ControllerNode";
}

RcCommand YAxisTiltVtolWidget::acrobatModeCommand() const
{
  return RcCommand::kAileElevRudThrottle;
}

RcCommand YAxisTiltVtolWidget::stabilizeModeCommand() const
{
  return RcCommand::kAngleSideSlipThrottleVector;
}

RcCommand YAxisTiltVtolWidget::loiterModeCommand() const
{
  return RcCommand::kPosVelAccPitchYawRollSideSlipMode;
}

YAML::Node YAxisTiltVtolWidget::staticParams() const
{
  return YAML::Node(YAML::NodeType::Map);
}

YAML::Node YAxisTiltVtolWidget::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void YAxisTiltVtolWidget::load(const YAML::Node&)
{
}

bool YAxisTiltVtolWidget::isValid()
{
  // TODO: Conditions on the number and signs of control surfaces.
  return true;
}
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
}  // namespace tobas
