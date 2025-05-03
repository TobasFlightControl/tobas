#include <QVBoxLayout>

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/controller/non_planar_pid.hpp"

namespace gui
{
namespace sa
{
NonPlanarPIDWidget::NonPlanarPIDWidget(
  RobotInfo& robot,
  const propulsion::PropulsionSystemWidget* propulsion_system,
  const fixed_wing::FixedWingWidget* fixed_wing)
  : robot_(robot), propulsion_system_(propulsion_system), fixed_wing_(fixed_wing)
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

const char* NonPlanarPIDWidget::name() const
{
  return "Non-Planar Multirotor PID";
}

const char* NonPlanarPIDWidget::description() const
{
  return "This is a PID controller for non-planar multirotors.";
}

QString NonPlanarPIDWidget::controllerPackage() const
{
  return "tobas_nonplanar_pid";
}

QString NonPlanarPIDWidget::pluginName() const
{
  return "ControllerNode";
}

tobas::rc_command_t NonPlanarPIDWidget::acrobatModeCommand() const
{
  return tobas::rc_command_t::ACCEL_RATE;
}

tobas::rc_command_t NonPlanarPIDWidget::stabilizeModeCommand() const
{
  return tobas::rc_command_t::ACCEL_ANGLE;
}

tobas::rc_command_t NonPlanarPIDWidget::loiterModeCommand() const
{
  return tobas::rc_command_t::POS_VEL_ANGLE;
}

YAML::Node NonPlanarPIDWidget::staticParams() const
{
  YAML::Node node(YAML::NodeType::Map);

  node["do_disturbance_compensation_translation"] = do_dist_comp_trans_->isChecked();
  node["do_disturbance_compensation_rotation"] = do_dist_comp_rot_->isChecked();

  return node;
}

YAML::Node NonPlanarPIDWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[do_dist_comp_trans_->text()] = do_dist_comp_trans_->isChecked();
  node[do_dist_comp_rot_->text()] = do_dist_comp_rot_->isChecked();

  return node;
}

void NonPlanarPIDWidget::load(const YAML::Node& node)
{
  do_dist_comp_trans_->setChecked(node[do_dist_comp_trans_->text()].as<bool>());
  do_dist_comp_rot_->setChecked(node[do_dist_comp_rot_->text()].as<bool>());
}

bool NonPlanarPIDWidget::isApplicable()
{
  // 固定翼を持たない
  if (fixed_wing_->hasFixedWing()) {
    return false;
  }

  // プロペラの個数条件
  if (propulsion_system_->numUnits() < kMinNumProp) {
    return false;
  }

  // 少なくとも1つのプロペラが鉛直上方向以外を向いている
  bool tilted_rotor_found = false;
  for (int i = 0; i < propulsion_system_->numUnits(); ++i) {
    const auto link_name = propulsion_system_->linkName(i);
    if (!robot_.isJntAxisAlwaysCollinear(link_name.toStdString(), kdl::Vector::UnitZ())) {
      tilted_rotor_found = true;
      break;
    }
  }
  if (!tilted_rotor_found) {
    return false;
  }

  return true;
}

bool NonPlanarPIDWidget::isValid()
{
  return true;
}
}  // namespace sa
}  // namespace gui
