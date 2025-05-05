#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/dynamics/dynamics.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/dynamics/manual.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
EngineDynamicsWidget::EngineDynamicsWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  method_name_ = new qt::ComboBox();
  methods_ = new qt::StackedWidget();

  rows->addWidget(method_name_);
  rows->addWidget(methods_);

  methods_->addWidget(new EngineDynamicsWidget_Manual());

  for (int i = 0; i < methods_->count(); ++i) {
    const auto method = qt::qPointerCast<EngineDynamicsWidget_Base>(methods_->widget(i));
    method_name_->addItem(method->name());
  }

  connect(
    method_name_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), methods_, &qt::StackedWidget::setCurrentIndex);
}

bool EngineDynamicsWidget::isValid()
{
  if (!selected()->isValid()) {
    return false;
  }

  return true;
}

YAML::Node EngineDynamicsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kMethodNameKey] = method_name_->currentText();

  for (int i = 0; i < methods_->count(); ++i) {
    const auto method = qt::qConstPointerCast<EngineDynamicsWidget_Base>(methods_->widget(i));
    node[method->name()] = method->dump();
  }

  return node;
}

void EngineDynamicsWidget::load(const YAML::Node& node)
{
  method_name_->setCurrentText(node[kMethodNameKey].as<QString>());

  for (int i = 0; i < methods_->count(); ++i) {
    const auto method = qt::qPointerCast<EngineDynamicsWidget_Base>(methods_->widget(i));
    method->load(node[method->name()]);
  }
}

double EngineDynamicsWidget::torqueConstant() const
{
  return selected()->torqueConstant();
}

double EngineDynamicsWidget::dynamicFrictionTorque() const
{
  return selected()->dynamicFrictionTorque();
}

EngineDynamicsWidget_Base* EngineDynamicsWidget::selected()
{
  return qt::qPointerCast<EngineDynamicsWidget_Base>(methods_->currentWidget());
}

const EngineDynamicsWidget_Base* EngineDynamicsWidget::selected() const
{
  return qt::qConstPointerCast<EngineDynamicsWidget_Base>(methods_->currentWidget());
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
