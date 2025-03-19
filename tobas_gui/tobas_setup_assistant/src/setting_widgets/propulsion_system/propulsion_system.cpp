#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/propulsion_system.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot)
{
  type_ = new qt::ComboBox();
  propulsions_ = new qt::StackedWidget();

  propulsions_->addWidget(new electric::PropulsionSystemWidget(node, robot));

  for (int i = 0; i < propulsions_->count(); ++i)
  {
    const auto propulsion = widget(i);
    type_->addItem(propulsion->name());

    connect(
      propulsion, &BasePropulsionSystemWidget::linkAdded,
      [this](const QString& link_name) { Q_EMIT linkAdded(link_name); });
    connect(
      propulsion, &BasePropulsionSystemWidget::linkRemoved,
      [this](const QString& link_name) { Q_EMIT linkRemoved(link_name); });
    connect(
      propulsion, &BasePropulsionSystemWidget::isTiltStateChanged,
      [this](const QString& link_name, bool is_tilt) { Q_EMIT isTiltStateChanged(link_name, is_tilt); });
    connect(
      propulsion, &BasePropulsionSystemWidget::tiltJointNameChanged,
      [this](const QString& link_name, const QString& tilt_joint_name)
      { Q_EMIT tiltJointNameChanged(link_name, tilt_joint_name); });
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

YAML::Node PropulsionSystemWidget::dump()
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
  return qobject_cast<BasePropulsionSystemWidget*>(propulsions_->widget(index));
}

BasePropulsionSystemWidget* PropulsionSystemWidget::selected()
{
  return qobject_cast<BasePropulsionSystemWidget*>(propulsions_->currentWidget());
}

const BasePropulsionSystemWidget* PropulsionSystemWidget::selected() const
{
  return qobject_cast<const BasePropulsionSystemWidget*>(propulsions_->currentWidget());
}
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
