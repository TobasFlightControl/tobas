#include "tobas_setup_assistant/setting_tabs/controller/non_planar_multicopter.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
namespace ctrl
{
NonPlanarMulticopterWidget::NonPlanarMulticopterWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  do_dist_comp_trans_ = new QCheckBox("Do Disturbance Compensation (Translation)");
  do_dist_comp_trans_->setChecked(false);
  rows->addWidget(do_dist_comp_trans_);

  do_dist_comp_rot_ = new QCheckBox("Do Disturbance Compensation (Rotation)");
  do_dist_comp_rot_->setChecked(false);
  rows->addWidget(do_dist_comp_rot_);

  rows->addStretch();
}

FrameType NonPlanarMulticopterWidget::frameType() const
{
  return FrameType::kNonPlanarMulticopter;
}

QString NonPlanarMulticopterWidget::controllerPackage() const
{
  return "tobas_nonplanar_multi_controller";
}

QString NonPlanarMulticopterWidget::pluginName() const
{
  return "tobas::nonplanar_multicopter::ControllerNode";
}

tobas::RcCommand NonPlanarMulticopterWidget::acrobatModeCommand() const
{
  return tobas::RcCommand::kAccelRate;
}

tobas::RcCommand NonPlanarMulticopterWidget::stabilizeModeCommand() const
{
  return tobas::RcCommand::kAccelAngle;
}

tobas::RcCommand NonPlanarMulticopterWidget::loiterModeCommand() const
{
  return tobas::RcCommand::kPosVelAccAngle;
}

YAML::Node NonPlanarMulticopterWidget::staticParams() const
{
  YAML::Node node(YAML::NodeType::Map);

  node["do_disturbance_compensation_translation"] = do_dist_comp_trans_->isChecked();
  node["do_disturbance_compensation_rotation"] = do_dist_comp_rot_->isChecked();

  return node;
}

YAML::Node NonPlanarMulticopterWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[do_dist_comp_trans_->text()] = do_dist_comp_trans_->isChecked();
  node[do_dist_comp_rot_->text()] = do_dist_comp_rot_->isChecked();

  return node;
}

void NonPlanarMulticopterWidget::load(const YAML::Node& node)
{
  do_dist_comp_trans_->setChecked(node[do_dist_comp_trans_->text()].as<bool>());
  do_dist_comp_rot_->setChecked(node[do_dist_comp_rot_->text()].as<bool>());
}

bool NonPlanarMulticopterWidget::isValid()
{
  return true;
}
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
