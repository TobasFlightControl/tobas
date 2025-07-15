#include "tobas_setup_assistant/setting_tabs/controller/multirotor_pid.hpp"

#include <QDebug>
#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
MultirotorPIDWidget::MultirotorPIDWidget(RobotInfo& robot) : robot_(robot)
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
  // 固定翼の操舵面をもたない
  if (robot_.uadf().control_surfaces.size() > 0) {
    return false;
  }

  // プロペラの個数条件
  if (robot_.uadf().thrusts.size() < kMinNumProp) {
    return false;
  }

  // Z軸正方向のプロペラのみ
  for (const auto& [joint_name, _] : robot_.uadf().thrusts) {
    const auto& link_name = robot_.linkName(joint_name);
    if (!robot_.isJntAxisAlwaysCollinear(link_name, kdl::Vector::UnitZ())) {
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
