#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/aerodynamics.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/manual.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/blade_theory.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/thrust_stand.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/uiuc.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
AerodynamicsWidget::AerodynamicsWidget(rclcpp::Node::SharedPtr node, PropellerWidget* propeller)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  method_name_ = new qt::ComboBox();
  methods_ = new qt::StackedWidget();

  rows->addWidget(method_name_);
  rows->addWidget(methods_);

  methods_->addWidget(new AerodynamicsWidget_Manual());
  methods_->addWidget(new AerodynamicsWidget_BladeTheory(propeller));
  methods_->addWidget(new AerodynamicsWidget_ThrustStand(node, propeller));
  methods_->addWidget(new AerodynamicsWidget_UIUC(node, propeller));

  for (int i = 0; i < methods_->count(); ++i)
  {
    const auto method = qt::qPointerCast<AerodynamicsWidget_Base>(methods_->widget(i));
    method_name_->addItem(method->name());
  }

  connect(
    method_name_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), methods_,
    &qt::StackedWidget::setCurrentIndex);
}

const char* AerodynamicsWidget::name() const
{
  return "Aerodynamics";
}

bool AerodynamicsWidget::isValid()
{
  if (!selected()->isValid())
    return false;

  return true;
}

void AerodynamicsWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<AerodynamicsWidget>(src);

  method_name_->setCurrentIndex(derived->method_name_->currentIndex());

  for (int i = 0; i < methods_->count(); ++i)
  {
    const auto des_method = qt::qPointerCast<AerodynamicsWidget_Base>(methods_->widget(i));
    const auto src_method = qt::qConstPointerCast<AerodynamicsWidget_Base>(derived->methods_->widget(i));
    des_method->copyFrom(src_method);
  }
}

YAML::Node AerodynamicsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kMethodNameKey] = method_name_->currentText();

  for (int i = 0; i < methods_->count(); ++i)
  {
    const auto method = qt::qConstPointerCast<AerodynamicsWidget_Base>(methods_->widget(i));
    node[method->name()] = method->dump();
  }

  return node;
}

void AerodynamicsWidget::load(const YAML::Node& node)
{
  method_name_->setCurrentText(node[kMethodNameKey].as<QString>());

  for (int i = 0; i < methods_->count(); ++i)
  {
    const auto method = qt::qPointerCast<AerodynamicsWidget_Base>(methods_->widget(i));
    method->load(node[method->name()]);
  }
}

double AerodynamicsWidget::motorConst() const
{
  return selected()->motorConst();
}

double AerodynamicsWidget::momentConst() const
{
  return selected()->momentConst();
}

double AerodynamicsWidget::dragConst() const
{
  return selected()->dragConst();
}

AerodynamicsWidget_Base* AerodynamicsWidget::selected()
{
  return qt::qPointerCast<AerodynamicsWidget_Base>(methods_->currentWidget());
}

const AerodynamicsWidget_Base* AerodynamicsWidget::selected() const
{
  return qt::qConstPointerCast<AerodynamicsWidget_Base>(methods_->currentWidget());
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
