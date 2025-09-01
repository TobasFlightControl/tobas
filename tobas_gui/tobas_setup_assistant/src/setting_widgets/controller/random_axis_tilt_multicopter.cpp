#include "tobas_setup_assistant/setting_tabs/controller/random_axis_tilt_multicopter.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
namespace ctrl
{
RandomAxisTiltMulticopterWidget::RandomAxisTiltMulticopterWidget()
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

FrameType RandomAxisTiltMulticopterWidget::frameType() const
{
  return FrameType::kRandomAxisTiltMulticopter;
}

QString RandomAxisTiltMulticopterWidget::controllerPackage() const
{
  return "tobas_random_axis_tilt_multi_controller";
}

QString RandomAxisTiltMulticopterWidget::pluginName() const
{
  return "tobas::random_axis_tilt_multicopter::ControllerNode";
}

tobas::RcCommand RandomAxisTiltMulticopterWidget::acrobatModeCommand() const
{
  return tobas::RcCommand::kAccelRate;
}

tobas::RcCommand RandomAxisTiltMulticopterWidget::stabilizeModeCommand() const
{
  return tobas::RcCommand::kAccelAngle;
}

tobas::RcCommand RandomAxisTiltMulticopterWidget::loiterModeCommand() const
{
  return tobas::RcCommand::kPosVelAngle;
}

YAML::Node RandomAxisTiltMulticopterWidget::staticParams() const
{
  YAML::Node node(YAML::NodeType::Map);

  node["do_disturbance_compensation_translation"] = do_dist_comp_trans_->isChecked();
  node["do_disturbance_compensation_rotation"] = do_dist_comp_rot_->isChecked();

  return node;
}

YAML::Node RandomAxisTiltMulticopterWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[do_dist_comp_trans_->text()] = do_dist_comp_trans_->isChecked();
  node[do_dist_comp_rot_->text()] = do_dist_comp_rot_->isChecked();

  return node;
}

void RandomAxisTiltMulticopterWidget::load(const YAML::Node& node)
{
  do_dist_comp_trans_->setChecked(node[do_dist_comp_trans_->text()].as<bool>());
  do_dist_comp_rot_->setChecked(node[do_dist_comp_rot_->text()].as<bool>());
}

bool RandomAxisTiltMulticopterWidget::isValid()
{
  return true;
}
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
