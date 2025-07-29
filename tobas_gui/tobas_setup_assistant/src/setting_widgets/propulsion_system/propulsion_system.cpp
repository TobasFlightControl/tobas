#include "tobas_setup_assistant/setting_tabs/propulsion_system/propulsion_system.hpp"

#include <QCheckBox>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf, Signals& sig)
  : sig_(sig)
{
  type_buttons_ = new QButtonGroup(this);
  propulsion_stack_ = new qt::StackedWidget();

  const auto eprop = new electric::PropulsionSystemWidget(node, uadf);
  const auto eprop_ckb = new QCheckBox(eprop->name());
  type_buttons_->addButton(eprop_ckb);
  type_buttons_->setId(eprop_ckb, kElectricId);
  propulsion_stack_->addWidget(eprop);

  const auto iprop = new ice::PropulsionSystemWidget(uadf);
  const auto iprop_ckb = new QCheckBox(iprop->name());
  type_buttons_->addButton(iprop_ckb);
  type_buttons_->setId(iprop_ckb, kIceId);
  propulsion_stack_->addWidget(iprop);

  eprop_ckb->setChecked(true);        // デフォルト
  type_buttons_->setExclusive(true);  // 1つのみ有効

  // Layout
  addWidget(eprop_ckb);
  addWidget(iprop_ckb);
  addSpacing(50);
  addWidget(propulsion_stack_);

  // Connection
  connect(type_buttons_, &QButtonGroup::idToggled, this, &self::onPropulsionTypeChanged);
}

const char* PropulsionSystemWidget::name() const
{
  return "Propulsion System";
}

const char* PropulsionSystemWidget::title() const
{
  return "Define Propulsion System";
}

const char* PropulsionSystemWidget::description() const
{
  return "Build the mathematical model for your propulsion system. "
         "Tobas supports two configurations:\n"
         "  1. Electric – battery‑powered with fixed‑pitch propellers\n"
         "  2. ICE – an internal‑combustion engine driving variable‑pitch propellers through gearboxes\n"
         "An accurate propulsion model is critical to maximizing aircraft performance. "
         "Select the appropriate architecture and enter the required parameters for each field.";
}

void PropulsionSystemWidget::updateInternalDataStructures()
{
  for (int i = 0; i < propulsion_stack_->count(); ++i) {
    const auto propulsion = widget(i);
    propulsion->updateInternalDataStructures();
  }
}

bool PropulsionSystemWidget::isValid()
{
  if (!selected()->isValid()) {
    return false;
  }

  return true;
}

YAML::Node PropulsionSystemWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  const auto type_button = type_buttons_->checkedButton();
  node[kTypeKey] = type_button->text();

  for (int i = 0; i < propulsion_stack_->count(); ++i) {
    const auto propulsion = widget(i);
    node[propulsion->name()] = propulsion->dump();
  }

  return node;
}

void PropulsionSystemWidget::load(const YAML::Node& node)
{
  const auto type_text = node[kTypeKey].as<QString>();
  for (const auto& button : type_buttons_->buttons()) {
    if (button->text() == type_text) {
      button->setChecked(true);
      break;
    }
  }

  for (int i = 0; i < propulsion_stack_->count(); ++i) {
    const auto propulsion = widget(i);
    propulsion->load(node[propulsion->name()]);
  }
}

tobas::PropulsionSystem PropulsionSystemWidget::type() const
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

BasePropulsionSystemWidget* PropulsionSystemWidget::widget(int index)
{
  return qt::qPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->widget(index));
}

const BasePropulsionSystemWidget* PropulsionSystemWidget::widget(int index) const
{
  return qt::qConstPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->widget(index));
}

BasePropulsionSystemWidget* PropulsionSystemWidget::selected()
{
  return qt::qPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->currentWidget());
}

const BasePropulsionSystemWidget* PropulsionSystemWidget::selected() const
{
  return qt::qConstPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->currentWidget());
}

void PropulsionSystemWidget::onPropulsionTypeChanged(int index)
{
  // 推進系のウィジェットを切り替える
  propulsion_stack_->setCurrentIndex(index);

  // 推進系の型が変わったことを他のウィジェットに通知
  Q_EMIT sig_.propulsionTypeChanged(widget(index)->type());
}
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
