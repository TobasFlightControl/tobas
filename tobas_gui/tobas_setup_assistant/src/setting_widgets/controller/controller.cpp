#include "tobas_setup_assistant/setting_tabs/controller/controller.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

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

  planar_multicopter_ = new PlanarMulticopterWidget();
  stack_->addWidget(planar_multicopter_);

  non_planar_multicopter_ = new NonPlanarMulticopterWidget();
  stack_->addWidget(non_planar_multicopter_);

  random_axis_tilt_multicopter_ = new RandomAxisTiltMulticopterWidget();
  stack_->addWidget(random_axis_tilt_multicopter_);

  fixed_wing_ = new FixedWingWidget();
  stack_->addWidget(fixed_wing_);
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
    node[controller->name()] = controller->dump();
  }

  return node;
}

void ControllerWidget::load(const YAML::Node& node)
{
  for (int i = 0; i < stack_->count(); ++i) {
    const auto controller = widget(i);
    controller->load(node[controller->name()]);
  }
}

void ControllerWidget::setFrameType(const FrameType& type)
{
  switch (type) {
    case FrameType::kUndefined:
      throw;  // TODO: 何かしら表示 (カスタムコントローラ？)
    case FrameType::kPlanarMulticopter:
      stack_->setCurrentWidget(planar_multicopter_);
      break;
    case FrameType::kNonPlanarMulticopter:
      stack_->setCurrentWidget(non_planar_multicopter_);
      break;
    case FrameType::kYAxisTiltMulticopter:
      throw;  // TODO
      break;
    case FrameType::kRandomAxisTiltMulticopter:
      stack_->setCurrentWidget(random_axis_tilt_multicopter_);
      break;
    case FrameType::kFixedWing:
      stack_->setCurrentWidget(fixed_wing_);
      break;
    default:
      throw;
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

bool ControllerWidget::isCommandCompatible(tobas::RcCommand command) const
{
  return command == acrobatModeCommand() || command == stabilizeModeCommand() || command == loiterModeCommand();
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
