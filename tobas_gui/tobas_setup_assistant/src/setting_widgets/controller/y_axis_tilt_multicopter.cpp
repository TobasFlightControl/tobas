#include "tobas_setup_assistant/setting_tabs/controller/y_axis_tilt_multicopter.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

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
  do_dist_comp_trans_->setChecked(false);
  rows->addWidget(do_dist_comp_trans_);

  do_dist_comp_rot_ = new QCheckBox("Do Disturbance Compensation (Rotation)");
  do_dist_comp_rot_->setChecked(false);
  rows->addWidget(do_dist_comp_rot_);

  rows->addStretch();
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

tobas::RcCommand YAxisTiltMulticopterWidget::acrobatModeCommand() const
{
  return tobas::RcCommand::kRateThrottleVector;
}

tobas::RcCommand YAxisTiltMulticopterWidget::stabilizeModeCommand() const
{
  return tobas::RcCommand::kAccelPitchYaw;
}

tobas::RcCommand YAxisTiltMulticopterWidget::loiterModeCommand() const
{
  return tobas::RcCommand::kPosVelAccPitchYaw;
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
