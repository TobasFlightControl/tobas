#include "tobas_setup_assistant/setting_tabs/controller/active_tilt_multicopter.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
namespace ctrl
{
ActiveTiltMulticopterWidget::ActiveTiltMulticopterWidget()
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

const char* ActiveTiltMulticopterWidget::name() const
{
  return "Active Tilt Multirotor PID";
}

QString ActiveTiltMulticopterWidget::controllerPackage() const
{
  return "tobas_tiltrotor_pid";
}

QString ActiveTiltMulticopterWidget::pluginName() const
{
  return "ControllerNode";
}

tobas::RcCommand ActiveTiltMulticopterWidget::acrobatModeCommand() const
{
  return tobas::RcCommand::kAccelRate;
}

tobas::RcCommand ActiveTiltMulticopterWidget::stabilizeModeCommand() const
{
  return tobas::RcCommand::kAccelAngle;
}

tobas::RcCommand ActiveTiltMulticopterWidget::loiterModeCommand() const
{
  return tobas::RcCommand::kPosVelAngle;
}

YAML::Node ActiveTiltMulticopterWidget::staticParams() const
{
  YAML::Node node(YAML::NodeType::Map);

  node["do_disturbance_compensation_translation"] = do_dist_comp_trans_->isChecked();
  node["do_disturbance_compensation_rotation"] = do_dist_comp_rot_->isChecked();

  return node;
}

YAML::Node ActiveTiltMulticopterWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[do_dist_comp_trans_->text()] = do_dist_comp_trans_->isChecked();
  node[do_dist_comp_rot_->text()] = do_dist_comp_rot_->isChecked();

  return node;
}

void ActiveTiltMulticopterWidget::load(const YAML::Node& node)
{
  do_dist_comp_trans_->setChecked(node[do_dist_comp_trans_->text()].as<bool>());
  do_dist_comp_rot_->setChecked(node[do_dist_comp_rot_->text()].as<bool>());
}

bool ActiveTiltMulticopterWidget::isValid()
{
  return true;
}
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
