#include "tobas_setup_assistant/setting_tabs/controller/controller.hpp"

#include <magic_enum/magic_enum.hpp>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/controller/custom.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/fixed_wing.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/non_planar_multicopter.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/planar_multicopter.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/random_axis_tilt_multicopter.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/y_axis_tilt_multicopter.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace ctrl
{
ControllerWidget::ControllerWidget()
{
  dont_use_builtin_ctrl_ = new QCheckBox("Do not use the built-in controller");

  stack_ = new qt::StackedWidget();

  stack_->addWidget(new CustomFrameWidget());
  stack_->addWidget(new PlanarMulticopterWidget());
  stack_->addWidget(new NonPlanarMulticopterWidget());
  stack_->addWidget(new YAxisTiltMulticopterWidget());
  stack_->addWidget(new RandomAxisTiltMulticopterWidget());
  stack_->addWidget(new FixedWingWidget());
  TOBAS_CHECK(static_cast<size_t>(stack_->count()) == magic_enum::enum_count<FrameType>());

  // Layout
  addWidget(stack_);
  addStretch();
  addWidget(dont_use_builtin_ctrl_);

  // Connection
  connect(dont_use_builtin_ctrl_, &QCheckBox::toggled, this, &self::onDontUseBuiltinCtrlCheckBoxToggled);
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

  node[dont_use_builtin_ctrl_->text()] = dont_use_builtin_ctrl_->isChecked();

  for (int i = 0; i < stack_->count(); ++i) {
    const auto controller = widget(i);
    node[textFromEnum(controller->frameType())] = controller->dump();
  }

  return node;
}

void ControllerWidget::load(const YAML::Node& node)
{
  dont_use_builtin_ctrl_->setChecked(node[dont_use_builtin_ctrl_->text()].as<bool>());

  for (int i = 0; i < stack_->count(); ++i) {
    const auto controller = widget(i);
    controller->load(node[textFromEnum(controller->frameType())]);
  }
}

void ControllerWidget::setFrameType(const FrameType& type)
{
  frame_type_ = type;

  // フレーム型が定義されていなければビルトイン制御器は使えない
  if (type == FrameType::kUndefined) {
    dont_use_builtin_ctrl_->setChecked(true);
    dont_use_builtin_ctrl_->setEnabled(false);
  }
  else {
    dont_use_builtin_ctrl_->setChecked(false);
    dont_use_builtin_ctrl_->setEnabled(true);
  }

  showCtrlWidgetWithFrameType(type);
}

bool ControllerWidget::useBuiltinContrller() const
{
  return !dont_use_builtin_ctrl_->isChecked();
}

QString ControllerWidget::controllerPackage() const
{
  return selected()->controllerPackage();
}

QString ControllerWidget::pluginName() const
{
  return selected()->pluginName();
}

RcCommand ControllerWidget::acrobatModeCommand() const
{
  return selected()->acrobatModeCommand();
}

RcCommand ControllerWidget::stabilizeModeCommand() const
{
  return selected()->stabilizeModeCommand();
}

RcCommand ControllerWidget::loiterModeCommand() const
{
  return selected()->loiterModeCommand();
}

YAML::Node ControllerWidget::staticParams() const
{
  return selected()->staticParams();
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

void ControllerWidget::showCtrlWidgetWithFrameType(const FrameType& type)
{
  for (int i = 0; i < stack_->count(); ++i) {
    if (widget(i)->frameType() == type) {
      stack_->setCurrentIndex(i);
      return;
    }
  }

  throw std::runtime_error("Controller widget not found for frame type: " + textFromEnum(type));
}

void ControllerWidget::onDontUseBuiltinCtrlCheckBoxToggled(bool checked)
{
  if (checked) {
    showCtrlWidgetWithFrameType(FrameType::kUndefined);
  }
  else {
    showCtrlWidgetWithFrameType(frame_type_);
  }
}
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
}  // namespace tobas
