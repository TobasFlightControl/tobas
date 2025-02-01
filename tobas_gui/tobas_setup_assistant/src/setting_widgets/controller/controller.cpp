#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/controller/controller.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/multirotor_pid.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/non_planar_pid.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/active_tilt_mr_pid.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/fixed_wing_lqr.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/custom.hpp"

namespace gui
{
namespace setup_assistant
{
ControllerWidget::ControllerWidget(
  RobotInfo& robot,
  const propulsion::PropulsionSystemWidget* propulsion_system,
  const fixed_wing::FixedWingWidget* fixed_wing)
  : robot_(robot), propulsion_system_(propulsion_system), fixed_wing_(fixed_wing)
{
  type_ = new qt::ComboBox();
  controllers_ = new qt::StackedWidget();
  description_ = new qt::DescriptionWidget("", kBodyPSize);

  addWidget(type_);
  addWidget(description_);
  addWidget(controllers_);

  controllers_->addWidget(new MultirotorPIDWidget(robot_, propulsion_system_, fixed_wing_));
  controllers_->addWidget(new NonPlanarPIDWidget(robot_, propulsion_system_, fixed_wing_));
  controllers_->addWidget(new ActiveTiltMultirotorPIDWidget(robot_, propulsion_system_, fixed_wing_));
  controllers_->addWidget(new FixedWingLQRWidget(robot_, propulsion_system_, fixed_wing_));
  controllers_->addWidget(new CustomControllerWidget());

  for (int i = 0; i < controllers_->count(); ++i)
  {
    const auto controller = qobject_cast<BaseControllerWidget*>(controllers_->widget(i));
    type_->addItem(controller->name());
  }

  connect(type_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), this, &self::setCurrentController);
  setCurrentController(0);
}

const char* ControllerWidget::name() const
{
  return "Controller";
}

const char* ControllerWidget::title() const
{
  return "Setup Controller";
}

const char* ControllerWidget::description() const
{
  return "Configure the flight controller by selecting one method and setting its parameters. "
         "You can fine-tune the parameters later, "
         "so it's acceptable to leave them at their default settings initially.";
}

void ControllerWidget::onOpened()
{
  // 現在の機体設定で適用可能な選択肢のみ選択可能にする
  for (int i = 0; i < controllers_->count(); ++i)
  {
    const auto controller = qobject_cast<BaseControllerWidget*>(controllers_->widget(i));
    type_->setItemEnabled(i, controller->isApplicable());
  }
}

void ControllerWidget::updateInternalDataStructures()
{
  return;
}

bool ControllerWidget::isValid()
{
  if (!selected()->isApplicable())
  {
    qt::qErrorBox(this, "The selected controller is not applicable to the airframe.");
    return false;
  }

  if (!selected()->isValid())
    return false;

  return true;
}

YAML::Node ControllerWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTypeKey] = type_->currentText();

  for (int i = 0; i < controllers_->count(); ++i)
  {
    const auto controller = qobject_cast<BaseControllerWidget*>(controllers_->widget(i));
    node[controller->name()] = controller->dump();
  }

  return node;
}

void ControllerWidget::load(const YAML::Node& node)
{
  type_->setCurrentText(node[kTypeKey].as<QString>());

  for (int i = 0; i < controllers_->count(); ++i)
  {
    const auto controller = qobject_cast<BaseControllerWidget*>(controllers_->widget(i));
    controller->load(node[controller->name()]);
  }
}

QString ControllerWidget::controllerPackage() const
{
  return selected()->controllerPackage();
}

QString ControllerWidget::pluginName() const
{
  return selected()->pluginName();
}

tobas::rc_command_t ControllerWidget::stabilizeModeCommand() const
{
  return selected()->stabilizeModeCommand();
}

tobas::rc_command_t ControllerWidget::acrobatModeCommand() const
{
  return selected()->acrobatModeCommand();
}

YAML::Node ControllerWidget::staticParams() const
{
  return selected()->staticParams();
}

bool ControllerWidget::isCommandCompatible(tobas::rc_command_t command) const
{
  return command == stabilizeModeCommand() || command == acrobatModeCommand();
}

void ControllerWidget::setCurrentController(int index)
{
  controllers_->setCurrentIndex(index);
  description_->setText(selected()->description());
}

BaseControllerWidget* ControllerWidget::selected()
{
  return qobject_cast<BaseControllerWidget*>(controllers_->currentWidget());
}

const BaseControllerWidget* ControllerWidget::selected() const
{
  return qobject_cast<BaseControllerWidget*>(controllers_->currentWidget());
}
}  // namespace setup_assistant
}  // namespace gui
