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

  connect(
    type_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), batteries_, &qt::StackedWidget::setCurrentIndex);

  auto lipo = new BatteryWidget_LiPo();
  type_->addItem(lipo->name());
  batteries_->addWidget(lipo);

  auto other = new BatteryWidget_Other();
  type_->addItem(other->name());
  batteries_->addWidget(other);
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
    auto battery = qobject_cast<BatteryWidget_Base*>(batteries_->widget(i));
    node[battery->name()] = battery->dump();
  }

  return node;
}

void BatteryWidget::load(const YAML::Node& node)
{
  type_->setCurrentText(node[kTypeKey].as<QString>());

  for (int i = 0; i < batteries_->count(); ++i)
  {
    auto battery = qobject_cast<BatteryWidget_Base*>(batteries_->widget(i));
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
}  // namespace setup_assistant
}  // namespace gui
