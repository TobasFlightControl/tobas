#include <QVBoxLayout>

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/controller/custom.hpp"

namespace gui
{
namespace sa
{
CustomControllerWidget::CustomControllerWidget()
  : command_map_{
      { kRateThrottleLabel, tobas::RATE_THROTTLE },
      { kAngleThrottleLabel, tobas::ANGLE_THROTTLE },
      { kPosVelAccYawLabel, tobas::POS_VEL_ACC_YAW },
      { kPoseTwistAccelLabel, tobas::POSE_TWIST_ACCEL },
      { kSpeedRollDeltaPitchLabel, tobas::SPEED_ROLL_DPITCH },
    }
{
  package_ = new ParamGetterWidget_LineEdit("Controller Package Name", "");
  plugin_ = new ParamGetterWidget_LineEdit("Controller Plugin Name", "");
  acrobat_mode_ = new ParamGetterWidget_ComboBox("Acrobat Mode", "");
  stabilize_mode_ = new ParamGetterWidget_ComboBox("Stabilize Mode", "");
  loiter_mode_ = new ParamGetterWidget_ComboBox("Loiter Mode", "");

  // Add command choices
  for (const auto& [text, _] : command_map_)
  {
    acrobat_mode_->addChoice(text);
    stabilize_mode_->addChoice(text);
    loiter_mode_->addChoice(text);
  }

  // Set default command
  acrobat_mode_->setValue(kRateThrottleLabel);
  stabilize_mode_->setValue(kAngleThrottleLabel);
  loiter_mode_->setValue(kPosVelAccYawLabel);

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(package_);
  rows->addWidget(plugin_);
  rows->addWidget(acrobat_mode_);
  rows->addWidget(stabilize_mode_);
  rows->addWidget(loiter_mode_);
  rows->addStretch();
  setLayout(rows);
}

const char* CustomControllerWidget::name() const
{
  return "Use Custom Controller";
}

const char* CustomControllerWidget::description() const
{
  return "";  // TODO: APIの案内など
}

QString CustomControllerWidget::controllerPackage() const
{
  return package_->getValue();
}

QString CustomControllerWidget::pluginName() const
{
  return plugin_->getValue();
}

tobas::rc_command_t CustomControllerWidget::acrobatModeCommand() const
{
  return command_map_.at(acrobat_mode_->getValue());
}

tobas::rc_command_t CustomControllerWidget::stabilizeModeCommand() const
{
  return command_map_.at(stabilize_mode_->getValue());
}

tobas::rc_command_t CustomControllerWidget::loiterModeCommand() const
{
  return command_map_.at(loiter_mode_->getValue());
}

YAML::Node CustomControllerWidget::staticParams() const
{
  return YAML::Node(YAML::NodeType::Map);
}

YAML::Node CustomControllerWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[package_->name()] = package_->getValue();
  node[plugin_->name()] = plugin_->getValue();
  node[acrobat_mode_->name()] = acrobat_mode_->getValue();
  node[stabilize_mode_->name()] = stabilize_mode_->getValue();
  node[loiter_mode_->name()] = loiter_mode_->getValue();

  return node;
}

void CustomControllerWidget::load(const YAML::Node& node)
{
  package_->setValue(node[package_->name()].as<QString>());
  plugin_->setValue(node[plugin_->name()].as<QString>());
  acrobat_mode_->setValue(node[acrobat_mode_->name()].as<QString>());
  stabilize_mode_->setValue(node[stabilize_mode_->name()].as<QString>());
  loiter_mode_->setValue(node[loiter_mode_->name()].as<QString>());
}

bool CustomControllerWidget::isApplicable()
{
  return true;
}

bool CustomControllerWidget::isValid()
{
  if (package_->getValue().isEmpty())
  {
    qt::qErrorBox(this, "Please specify custom controller package name.");
    return false;
  }

  if (plugin_->getValue().isEmpty())
  {
    qt::qErrorBox(this, "Please specify custom controller plugin name.");
    return false;
  }

  return true;
}
}  // namespace sa
}  // namespace gui
