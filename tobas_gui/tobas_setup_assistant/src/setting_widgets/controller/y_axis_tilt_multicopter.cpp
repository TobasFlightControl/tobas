// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/controller/y_axis_tilt_multicopter.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace ctrl
{
YAxisTiltMulticopterWidget::YAxisTiltMulticopterWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  do_dist_comp_trans_ = new QCheckBox("Do Disturbance Compensation (Translation)");
  rows->addWidget(do_dist_comp_trans_);

  do_dist_comp_rot_ = new QCheckBox("Do Disturbance Compensation (Rotation)");
  rows->addWidget(do_dist_comp_rot_);

  rows->addStretch();
}

void YAxisTiltMulticopterWidget::setToDefaults()
{
  do_dist_comp_trans_->setChecked(false);
  do_dist_comp_rot_->setChecked(false);
}

FrameType YAxisTiltMulticopterWidget::frameType() const
{
  return FrameType::kYAxisTiltMulticopter;
}

QString YAxisTiltMulticopterWidget::controllerPackage() const
{
  return "tobas_y_axis_tilt_multi_controller";
}

QString YAxisTiltMulticopterWidget::pluginName() const
{
  return "tobas::y_axis_tilt_multicopter::ControllerNode";
}

RcCommand YAxisTiltMulticopterWidget::acrobatModeCommand() const
{
  return RcCommand::kRateThrottleVector;
}

RcCommand YAxisTiltMulticopterWidget::stabilizeModeCommand() const
{
  return RcCommand::kAccelPitchYaw;
}

RcCommand YAxisTiltMulticopterWidget::loiterModeCommand() const
{
  return RcCommand::kPosVelAccPitchYaw;
}

YAML::Node YAxisTiltMulticopterWidget::staticParams() const
{
  YAML::Node node(YAML::NodeType::Map);

  node["do_disturbance_compensation_translation"] = do_dist_comp_trans_->isChecked();
  node["do_disturbance_compensation_rotation"] = do_dist_comp_rot_->isChecked();

  return node;
}

YAML::Node YAxisTiltMulticopterWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[do_dist_comp_trans_->text()] = do_dist_comp_trans_->isChecked();
  node[do_dist_comp_rot_->text()] = do_dist_comp_rot_->isChecked();

  return node;
}

void YAxisTiltMulticopterWidget::load(const YAML::Node& node)
{
  do_dist_comp_trans_->setChecked(node[do_dist_comp_trans_->text()].as<bool>());
  do_dist_comp_rot_->setChecked(node[do_dist_comp_rot_->text()].as<bool>());
}

bool YAxisTiltMulticopterWidget::isValid()
{
  return true;
}
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
}  // namespace tobas
