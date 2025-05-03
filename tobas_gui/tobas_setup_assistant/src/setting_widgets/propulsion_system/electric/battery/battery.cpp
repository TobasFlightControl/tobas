#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/battery/battery.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/battery/lipo.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/battery/other.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
BatteryWidget::BatteryWidget()
{
  type_ = new qt::ComboBox();
  batteries_ = new qt::StackedWidget();

  batteries_->addWidget(new BatteryWidget_LiPo());
  batteries_->addWidget(new BatteryWidget_Other());

  for (int i = 0; i < batteries_->count(); ++i) {
    const auto battery = widget(i);
    type_->addItem(battery->name());
  }

  const auto rows = new QVBoxLayout();
  rows->addWidget(type_);
  rows->addWidget(batteries_);
  setLayout(rows);

  connect(
    type_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), batteries_, &qt::StackedWidget::setCurrentIndex);
}

bool BatteryWidget::isValid()
{
  if (!selected()->isValid()) {
    return false;
  }

  return true;
}

YAML::Node BatteryWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTypeKey] = type_->currentText();

  for (int i = 0; i < batteries_->count(); ++i) {
    const auto battery = widget(i);
    node[battery->name()] = battery->dump();
  }

  return node;
}

void BatteryWidget::load(const YAML::Node& node)
{
  type_->setCurrentText(node[kTypeKey].as<QString>());

  for (int i = 0; i < batteries_->count(); ++i) {
    const auto battery = widget(i);
    battery->load(node[battery->name()]);
  }
}

double BatteryWidget::nominalVoltage()
{
  return selected()->nominalVoltage();
}

double BatteryWidget::maxVoltage()
{
  return selected()->maxVoltage();
}

double BatteryWidget::sagVoltage()
{
  return selected()->sagVoltage();
}

double BatteryWidget::maxCurrent()
{
  return selected()->maxCurrent();
}

double BatteryWidget::capacity()
{
  return selected()->capacity();
}

double BatteryWidget::internalRegistance()
{
  return selected()->internalRegistance();
}

BatteryWidget_Base* BatteryWidget::widget(int index)
{
  return qt::qPointerCast<BatteryWidget_Base>(batteries_->widget(index));
}

const BatteryWidget_Base* BatteryWidget::widget(int index) const
{
  return qt::qConstPointerCast<BatteryWidget_Base>(batteries_->widget(index));
}

BatteryWidget_Base* BatteryWidget::selected()
{
  return qt::qPointerCast<BatteryWidget_Base>(batteries_->currentWidget());
}

const BatteryWidget_Base* BatteryWidget::selected() const
{
  return qt::qConstPointerCast<BatteryWidget_Base>(batteries_->currentWidget());
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
