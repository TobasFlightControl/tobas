#include "tobas_setup_assistant/setting_tabs/controller/active_tilt_mr_pid.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
ActiveTiltMultirotorPIDWidget::ActiveTiltMultirotorPIDWidget(RobotInfo& robot) : robot_(robot)
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

const char* ActiveTiltMultirotorPIDWidget::name() const
{
  return "Active Tilt Multirotor PID";
}

QString ActiveTiltMultirotorPIDWidget::controllerPackage() const
{
  return "tobas_tiltrotor_pid";
}

QString ActiveTiltMultirotorPIDWidget::pluginName() const
{
  return "ControllerNode";
}

tobas::rc_command_t ActiveTiltMultirotorPIDWidget::acrobatModeCommand() const
{
  return tobas::rc_command_t::ACCEL_RATE;
}

tobas::rc_command_t ActiveTiltMultirotorPIDWidget::stabilizeModeCommand() const
{
  return tobas::rc_command_t::ACCEL_ANGLE;
}

tobas::rc_command_t ActiveTiltMultirotorPIDWidget::loiterModeCommand() const
{
  return tobas::rc_command_t::POS_VEL_ANGLE;
}

YAML::Node ActiveTiltMultirotorPIDWidget::staticParams() const
{
  YAML::Node node(YAML::NodeType::Map);

  node["do_disturbance_compensation_translation"] = do_dist_comp_trans_->isChecked();
  node["do_disturbance_compensation_rotation"] = do_dist_comp_rot_->isChecked();

  return node;
}

YAML::Node ActiveTiltMultirotorPIDWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[do_dist_comp_trans_->text()] = do_dist_comp_trans_->isChecked();
  node[do_dist_comp_rot_->text()] = do_dist_comp_rot_->isChecked();

  return node;
}

void ActiveTiltMultirotorPIDWidget::load(const YAML::Node& node)
{
  do_dist_comp_trans_->setChecked(node[do_dist_comp_trans_->text()].as<bool>());
  do_dist_comp_rot_->setChecked(node[do_dist_comp_rot_->text()].as<bool>());
}

bool ActiveTiltMultirotorPIDWidget::isApplicable()
{
  // 固定翼の操舵面をもたない
  if (robot_.uadf().control_surfaces.size() > 0) {
    return false;
  }

  // プロペラの個数条件
  if (robot_.uadf().thrusts.size() < kMinNumProp) {
    return false;
  }

  // 全てティルトロータ
  // TODO: アクティブティルトと固定モータの混合モデルも許容
  if (robot_.uadf().thrusts.size() != robot_.uadf().tilts.size()) {
    return false;
  }

  // TODO: ティルト軸とロータ軸が直行する (cf. tobas_drone_tools/tr_mixer_pinv)
  // TODO: プロペラリンクとティルト軸の距離が閾値以下

  return true;
}

bool ActiveTiltMultirotorPIDWidget::isValid()
{
  return true;
}
}  // namespace sa
}  // namespace gui
