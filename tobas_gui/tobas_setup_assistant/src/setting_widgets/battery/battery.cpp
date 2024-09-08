#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/battery/battery.hpp"
#include "tobas_setup_assistant/setting_tabs/battery/lipo.hpp"
#include "tobas_setup_assistant/setting_tabs/battery/other.hpp"

namespace gui
{
namespace setup_assistant
{
const char* BatteryWidget::name() const
{
  return "Battery";
}

const char* BatteryWidget::title() const
{
  return "Define Battery";
}

const char* BatteryWidget::description() const
{
  return "Configure the LiPo (Lithium Polymer) battery settings. "
         "It is assumed that a single battery will power all motors. "
         "Therefore, the settings here will affect the control of all motors.";
}

void BatteryWidget::onInit()
{
  type_ = new qt::ComboBox();
  batteries_ = new qt::StackedWidget();

  addWidget(type_);
  addWidget(batteries_);

  batteries_->addWidget(new BatteryWidget_LiPo());
  batteries_->addWidget(new BatteryWidget_Other());

  for (int i = 0; i < batteries_->count(); ++i)
  {
    const auto battery = qobject_cast<BatteryWidget_Base*>(batteries_->widget(i));
    type_->addItem(battery->name());
  }

  connect(
    type_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), batteries_, &qt::StackedWidget::setCurrentIndex);
}

void BatteryWidget::onOpened()
{
  return;
}

void BatteryWidget::updateInternalDataStructures()
{
  return;
}

bool BatteryWidget::isValid()
{
  if (!selected()->isValid())
    return false;

  return true;
}

YAML::Node BatteryWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTypeKey] = type_->currentText();

  for (int i = 0; i < batteries_->count(); ++i)
  {
    const auto battery = qobject_cast<BatteryWidget_Base*>(batteries_->widget(i));
    node[battery->name()] = battery->dump();
  }

  return node;
}

void BatteryWidget::load(const YAML::Node& node)
{
  type_->setCurrentText(node[kTypeKey].as<QString>());

  for (int i = 0; i < batteries_->count(); ++i)
  {
    const auto battery = qobject_cast<BatteryWidget_Base*>(batteries_->widget(i));
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

BatteryWidget_Base* BatteryWidget::selected()
{
  return qobject_cast<BatteryWidget_Base*>(batteries_->currentWidget());
}

const BatteryWidget_Base* BatteryWidget::selected() const
{
  return qobject_cast<BatteryWidget_Base*>(batteries_->currentWidget());
}
}  // namespace setup_assistant
}  // namespace gui
