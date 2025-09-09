#include "tobas_setup_assistant/setting_tabs/controller/planar_multicopter.hpp"

#include <QDebug>
#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
namespace ctrl
{
PlanarMulticopterWidget::PlanarMulticopterWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  do_dist_comp_trans_ = new QCheckBox("Do Disturbance Compensation (Translation)");
  do_dist_comp_trans_->setChecked(false);
  rows->addWidget(do_dist_comp_trans_);

  do_dist_comp_rot_ = new QCheckBox("Do Disturbance Compensation (Rotation)");
  do_dist_comp_rot_->setChecked(false);
  rows->addWidget(do_dist_comp_rot_);

  standard_second_order_form_tuning_ = new QCheckBox("Standard Second-Order Form Tuning");
  standard_second_order_form_tuning_->setChecked(true);
  rows->addWidget(standard_second_order_form_tuning_);

  rows->addStretch();
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

tobas::RcCommand PlanarMulticopterWidget::acrobatModeCommand() const
{
  return tobas::RcCommand::kRateThrottle;
}

tobas::RcCommand PlanarMulticopterWidget::stabilizeModeCommand() const
{
  return tobas::RcCommand::kAccelYaw;
}

tobas::RcCommand PlanarMulticopterWidget::loiterModeCommand() const
{
  return tobas::RcCommand::kPosVelYaw;
}

YAML::Node PlanarMulticopterWidget::staticParams() const
{
  YAML::Node node(YAML::NodeType::Map);

  node["do_disturbance_compensation_translation"] = do_dist_comp_trans_->isChecked();
  node["do_disturbance_compensation_rotation"] = do_dist_comp_rot_->isChecked();
  node["standard_second_order_form_tuning"] = standard_second_order_form_tuning_->isChecked();

  return node;
}

YAML::Node PlanarMulticopterWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[do_dist_comp_trans_->text()] = do_dist_comp_trans_->isChecked();
  node[do_dist_comp_rot_->text()] = do_dist_comp_rot_->isChecked();
  node[standard_second_order_form_tuning_->text()] = standard_second_order_form_tuning_->isChecked();

  return node;
}

void PlanarMulticopterWidget::load(const YAML::Node& node)
{
  do_dist_comp_trans_->setChecked(node[do_dist_comp_trans_->text()].as<bool>());
  do_dist_comp_rot_->setChecked(node[do_dist_comp_rot_->text()].as<bool>());
  standard_second_order_form_tuning_->setChecked(node[standard_second_order_form_tuning_->text()].as<bool>());
}

bool PlanarMulticopterWidget::isValid()
{
  return true;
}
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
