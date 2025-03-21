#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/propulsion_system.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot, Signals& _signals)
{
  type_ = new qt::ComboBox();
  propulsions_ = new qt::StackedWidget();

  propulsions_->addWidget(new electric::PropulsionSystemWidget(node, robot, _signals));
  propulsions_->addWidget(new ice::PropulsionSystemWidget(node, robot, _signals));

  for (int i = 0; i < propulsions_->count(); ++i)
  {
    const auto propulsion = widget(i);
    type_->addItem(propulsion->name());
  }

  addWidget(type_);
  addSpacing(50);
  addWidget(propulsions_);

  connect(
    type_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), propulsions_, &qt::StackedWidget::setCurrentIndex);
}

const char* PropulsionSystemWidget::name() const
{
  return "Propulsion";
}

const char* PropulsionSystemWidget::title() const
{
  return "Define Propulsion System";
}

const char* PropulsionSystemWidget::description() const
{
  return "";  // TODO
}

void PropulsionSystemWidget::onOpened()
{
}

void PropulsionSystemWidget::updateInternalDataStructures()
{
  for (int i = 0; i < propulsions_->count(); ++i)
  {
    const auto propulsion = widget(i);
    propulsion->updateInternalDataStructures();
  }
}

bool PropulsionSystemWidget::isValid()
{
  if (!selected()->isValid())
    return false;

  return true;
}

YAML::Node PropulsionSystemWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTypeKey] = type_->currentText();

  for (int i = 0; i < propulsions_->count(); ++i)
  {
    const auto propulsion = widget(i);
    node[propulsion->name()] = propulsion->dump();
  }

  return node;
}

void PropulsionSystemWidget::load(const YAML::Node& node)
{
  type_->setCurrentText(node[kTypeKey].as<QString>());

  for (int i = 0; i < propulsions_->count(); ++i)
  {
    const auto propulsion = widget(i);
    propulsion->load(node[propulsion->name()]);
  }
}

tobas::propulsion_system_t PropulsionSystemWidget::type() const
{
  return selected()->type();
}

int PropulsionSystemWidget::numUnits() const
{
  return selected()->numUnits();
}

QString PropulsionSystemWidget::linkName(int index) const
{
  return selected()->linkName(index);
}

bool PropulsionSystemWidget::isTiltRotor(int index) const
{
  return selected()->isTiltRotor(index);
}

QString PropulsionSystemWidget::tiltJointName(int index) const
{
  return selected()->tiltJointName(index);
}

BasePropulsionSystemWidget* PropulsionSystemWidget::widget(int index)
{
  return qt::qPointerCast<BasePropulsionSystemWidget>(propulsions_->widget(index));
}

const BasePropulsionSystemWidget* PropulsionSystemWidget::widget(int index) const
{
  return qt::qConstPointerCast<BasePropulsionSystemWidget>(propulsions_->widget(index));
}

BasePropulsionSystemWidget* PropulsionSystemWidget::selected()
{
  return qt::qPointerCast<BasePropulsionSystemWidget>(propulsions_->currentWidget());
}

const BasePropulsionSystemWidget* PropulsionSystemWidget::selected() const
{
  return qt::qConstPointerCast<BasePropulsionSystemWidget>(propulsions_->currentWidget());
}
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
