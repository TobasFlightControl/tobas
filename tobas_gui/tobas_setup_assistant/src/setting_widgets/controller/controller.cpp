#include "tobas_setup_assistant/setting_tabs/controller/controller.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/controller/fixed_wing.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/non_planar_multicopter.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/planar_multicopter.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/random_axis_tilt_multicopter.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/y_axis_tilt_multicopter.hpp"

namespace gui
{
namespace sa
{
namespace ctrl
{
ControllerWidget::ControllerWidget()
{
  stack_ = new qt::StackedWidget();
  addWidget(stack_);

  stack_->addWidget(new PlanarMulticopterWidget());
  stack_->addWidget(new NonPlanarMulticopterWidget());
  stack_->addWidget(new YAxisTiltMulticopterWidget());
  stack_->addWidget(new RandomAxisTiltMulticopterWidget());
  stack_->addWidget(new FixedWingWidget());
}

const char* ControllerWidget::name() const
{
  return "Flight Controller";
}

const char* ControllerWidget::title() const
{
  return "Set up Flight Controller";
}

const char* ControllerWidget::description() const
{
  return "Configure the flight controller algorithm. "
         "Setup Assistant analyzed your UADF, and a controller suited to the airframe has been assigned automatically. "
         "The static parameters of the controller are listed below.";
}

void ControllerWidget::updateInternalDataStructures()
{
}

bool ControllerWidget::isValid()
{
  if (!selected()->isValid()) {
    return false;
  }

  return true;
}

YAML::Node ControllerWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < stack_->count(); ++i) {
    const auto controller = widget(i);
    node[textFromEnum(controller->frameType())] = controller->dump();
  }

  return node;
}

void ControllerWidget::load(const YAML::Node& node)
{
  for (int i = 0; i < stack_->count(); ++i) {
    const auto controller = widget(i);
    controller->load(node[textFromEnum(controller->frameType())]);
  }
}

FrameType ControllerWidget::getFrameType() const
{
  return selected()->frameType();
}

void ControllerWidget::setFrameType(const FrameType& type)
{
  for (int i = 0; i < stack_->count(); ++i) {
    if (widget(i)->frameType() == type) {
      stack_->setCurrentIndex(i);
      return;
    }
  }

  throw std::runtime_error("Controller not found for frame type: " + textFromEnum(type));
}

QString ControllerWidget::controllerPackage() const
{
  return selected()->controllerPackage();
}

QString ControllerWidget::pluginName() const
{
  return selected()->pluginName();
}

tobas::RcCommand ControllerWidget::acrobatModeCommand() const
{
  return selected()->acrobatModeCommand();
}

tobas::RcCommand ControllerWidget::stabilizeModeCommand() const
{
  return selected()->stabilizeModeCommand();
}

tobas::RcCommand ControllerWidget::loiterModeCommand() const
{
  return selected()->loiterModeCommand();
}

YAML::Node ControllerWidget::staticParams() const
{
  return selected()->staticParams();
}

void ControllerWidget::setCurrentController(int index)
{
  stack_->setCurrentIndex(index);
}

BaseControllerWidget* ControllerWidget::widget(int index)
{
  return qt::qPointerCast<BaseControllerWidget>(stack_->widget(index));
}

const BaseControllerWidget* ControllerWidget::widget(int index) const
{
  return qt::qConstPointerCast<BaseControllerWidget>(stack_->widget(index));
}

BaseControllerWidget* ControllerWidget::selected()
{
  return qt::qPointerCast<BaseControllerWidget>(stack_->currentWidget());
}

const BaseControllerWidget* ControllerWidget::selected() const
{
  return qt::qConstPointerCast<BaseControllerWidget>(stack_->currentWidget());
}
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
