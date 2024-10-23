#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electrodynamics/no_select.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electrodynamics/electrodynamics.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electrodynamics/spec.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electrodynamics/experiment.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
ElectrodynamicsWidget::ElectrodynamicsWidget(
  rclcpp::Node::SharedPtr node,
  MotorWidget* motor,
  AerodynamicsWidget* aerodynamics)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  method_name_ = new qt::ComboBox();
  methods_ = new qt::StackedWidget();

  rows->addWidget(method_name_);
  rows->addWidget(methods_);

  methods_->addWidget(new ElectrodynamicsWidget_NoSelect());
  methods_->addWidget(new ElectrodynamicsWidget_Spec(motor, aerodynamics));
  methods_->addWidget(new ElectroDynamicsWidget_Experiment(node));

  for (int i = 0; i < methods_->count(); ++i)
  {
    const auto method = qobject_cast<ElectrodynamicsWidget_Base*>(methods_->widget(i));
    method_name_->addItem(method->name());
  }

  connect(
    method_name_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), methods_,
    &qt::StackedWidget::setCurrentIndex);
}

const char* ElectrodynamicsWidget::name() const
{
  return "Electrodynamics";
}

bool ElectrodynamicsWidget::isValid()
{
  if (!selected()->isValid())
    return false;

  return true;
}

void ElectrodynamicsWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qobject_cast<const ElectrodynamicsWidget*>(src);

  method_name_->setCurrentIndex(derived->method_name_->currentIndex());

  for (int i = 0; i < methods_->count(); ++i)
  {
    const auto des_method = qobject_cast<ElectrodynamicsWidget_Base*>(methods_->widget(i));
    const auto src_method = qobject_cast<ElectrodynamicsWidget_Base*>(derived->methods_->widget(i));
    des_method->copyFrom(src_method);
  }
}

YAML::Node ElectrodynamicsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kMethodNameKey] = method_name_->currentText();

  for (int i = 0; i < methods_->count(); ++i)
  {
    const auto method = qobject_cast<ElectrodynamicsWidget_Base*>(methods_->widget(i));
    node[method->name()] = method->dump();
  }

  return node;
}

void ElectrodynamicsWidget::load(const YAML::Node& node)
{
  method_name_->setCurrentText(node[kMethodNameKey].as<QString>());

  for (int i = 0; i < methods_->count(); ++i)
  {
    const auto method = qobject_cast<ElectrodynamicsWidget_Base*>(methods_->widget(i));
    method->load(node[method->name()]);
  }
}

std::pair<double, double> ElectrodynamicsWidget::rotSpeedCoefs() const
{
  return selected()->rotSpeedCoefs();
}

ElectrodynamicsWidget_Base* ElectrodynamicsWidget::selected()
{
  return qobject_cast<ElectrodynamicsWidget_Base*>(methods_->currentWidget());
}

const ElectrodynamicsWidget_Base* ElectrodynamicsWidget::selected() const
{
  return qobject_cast<ElectrodynamicsWidget_Base*>(methods_->currentWidget());
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
