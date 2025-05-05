#include "tobas_setup_assistant/setting_tabs/controller/multirotor_pid.hpp"

#include <QVBoxLayout>
#include <QDebug>

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/message.hpp>

namespace gui
{
namespace sa
{
MultirotorPIDWidget::MultirotorPIDWidget(
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

const char* MultirotorPIDWidget::name() const
{
  return "Multirotor PID";
}

const char* MultirotorPIDWidget::description() const
{
  return "This controller for planar multirotors employs PID for both position and attitude control.";
}

QString MultirotorPIDWidget::controllerPackage() const
{
  return "tobas_multirotor_pid";
}

QString MultirotorPIDWidget::pluginName() const
{
  return "ControllerNode";
}

tobas::rc_command_t MultirotorPIDWidget::acrobatModeCommand() const
{
  return tobas::rc_command_t::RATE_THROTTLE;
}

tobas::rc_command_t MultirotorPIDWidget::stabilizeModeCommand() const
{
  return tobas::rc_command_t::ACCEL_YAW;
}

tobas::rc_command_t MultirotorPIDWidget::loiterModeCommand() const
{
  return tobas::rc_command_t::POS_VEL_YAW;
}

YAML::Node MultirotorPIDWidget::staticParams() const
{
  YAML::Node node(YAML::NodeType::Map);

  node["do_disturbance_compensation_translation"] = do_dist_comp_trans_->isChecked();
  node["do_disturbance_compensation_rotation"] = do_dist_comp_rot_->isChecked();

  return node;
}

YAML::Node MultirotorPIDWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[do_dist_comp_trans_->text()] = do_dist_comp_trans_->isChecked();
  node[do_dist_comp_rot_->text()] = do_dist_comp_rot_->isChecked();

  return node;
}

void MultirotorPIDWidget::load(const YAML::Node& node)
{
  do_dist_comp_trans_->setChecked(node[do_dist_comp_trans_->text()].as<bool>());
  do_dist_comp_rot_->setChecked(node[do_dist_comp_rot_->text()].as<bool>());
}

bool MultirotorPIDWidget::isApplicable()
{
  // 固定翼を持たない
  if (fixed_wing_->hasFixedWing()) {
    return false;
  }

  // プロペラの個数条件
  if (propulsion_system_->numUnits() < kMinNumProp) {
    return false;
  }

  // Z軸正方向のプロペラのみ
  for (int i = 0; i < propulsion_system_->numUnits(); ++i) {
    const auto link_name = propulsion_system_->linkName(i);
    if (!robot_.isJntAxisAlwaysCollinear(link_name.toStdString(), kdl::Vector::UnitZ())) {
      return false;
    }
  }

  return true;
}

bool MultirotorPIDWidget::isValid()
{
  return true;
}
}  // namespace sa
}  // namespace gui
