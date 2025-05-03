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
  type_buttons_ = new QButtonGroup(this);
  propulsion_stack_ = new qt::StackedWidget();

  const auto eprop = new electric::PropulsionSystemWidget(node, robot, _signals);
  const auto eprop_ckb = new QCheckBox(eprop->name());
  type_buttons_->addButton(eprop_ckb);
  type_buttons_->setId(eprop_ckb, kElectricId);
  propulsion_stack_->addWidget(eprop);

  const auto iprop = new ice::PropulsionSystemWidget(node, robot, _signals);
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
  // 全ての設定をリセット
  for (int i = 0; i < propulsion_stack_->count(); ++i) {
    const auto propulsion = widget(i);
    propulsion->reset();
  }

  propulsion_stack_->setCurrentIndex(index);
}
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
