// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/controller/planar_multicopter.hpp"

#include <QVBoxLayout>

#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas::gui::sa::ctrl
{
PlanarMulticopterWidget::PlanarMulticopterWidget()
{
  const auto rows = new QVBoxLayout(this);

  use_additional_acceleration_ = new QCheckBox("Use Additional Acceleration");
  rows->addWidget(use_additional_acceleration_);

  do_dist_comp_trans_ = new QCheckBox("Do Disturbance Compensation (Translation)");
  rows->addWidget(do_dist_comp_trans_);

  do_dist_comp_rot_ = new QCheckBox("Do Disturbance Compensation (Rotation)");
  rows->addWidget(do_dist_comp_rot_);

  rows->addStretch();
}

void PlanarMulticopterWidget::setToDefaults()
{
  use_additional_acceleration_->setChecked(false);
  do_dist_comp_trans_->setChecked(false);
  do_dist_comp_rot_->setChecked(false);
}

FrameType PlanarMulticopterWidget::frameType() const
{
  return FrameType::kPlanarMulticopter;
}

QString PlanarMulticopterWidget::controllerPackage() const
{
  return "tobas_planar_multi_controller";
}

QString PlanarMulticopterWidget::pluginName() const
{
  return "tobas::planar_multicopter::ControllerNode";
}

RcCommand PlanarMulticopterWidget::acrobatModeCommand() const
{
  return RcCommand::kRateThrottle;
}

RcCommand PlanarMulticopterWidget::stabilizeModeCommand() const
{
  return RcCommand::kAngleThrottle;
}

RcCommand PlanarMulticopterWidget::loiterModeCommand() const
{
  return RcCommand::kPosVelAccYaw;
}

YAML::Node PlanarMulticopterWidget::staticParams() const
{
  YAML::Node node(YAML::NodeType::Map);

  node["use_additional_acceleration"] = use_additional_acceleration_->isChecked();
  node["do_disturbance_compensation_translation"] = do_dist_comp_trans_->isChecked();
  node["do_disturbance_compensation_rotation"] = do_dist_comp_rot_->isChecked();

  return node;
}

YAML::Node PlanarMulticopterWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[use_additional_acceleration_->text()] = use_additional_acceleration_->isChecked();
  node[do_dist_comp_trans_->text()] = do_dist_comp_trans_->isChecked();
  node[do_dist_comp_rot_->text()] = do_dist_comp_rot_->isChecked();

  return node;
}

void PlanarMulticopterWidget::load(const YAML::Node& _node)
{
  use_additional_acceleration_->setChecked(_node[use_additional_acceleration_->text()].as<bool>());
  do_dist_comp_trans_->setChecked(_node[do_dist_comp_trans_->text()].as<bool>());
  do_dist_comp_rot_->setChecked(_node[do_dist_comp_rot_->text()].as<bool>());
}

bool PlanarMulticopterWidget::isValid()
{
  return true;
}
}  // namespace tobas::gui::sa::ctrl
