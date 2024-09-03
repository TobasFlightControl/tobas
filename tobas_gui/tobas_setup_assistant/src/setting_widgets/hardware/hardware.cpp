#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/hardware/hardware.hpp"
#include "tobas_setup_assistant/setting_tabs/hardware/aso.hpp"

namespace gui
{
namespace setup_assistant
{
HardwareWidget::HardwareWidget()
{
}

const char* HardwareWidget::name() const
{
  return "Hardware";
}

const char* HardwareWidget::title() const
{
  return "Select Flight Controller Hardware";
}

const char* HardwareWidget::description() const
{
  return "";  // TODO
}

void HardwareWidget::onInit()
{
  type_ = new qt::ComboBox();
  hardwares_ = new qt::StackedWidget();
  description_ = new qt::DescriptionWidget("", kBodyPSize);

  addWidget(type_);
  addWidget(description_);
  addWidget(hardwares_);

  hardwares_->addWidget(new AsoWidget());

  for (int i = 0; i < hardwares_->count(); ++i)
  {
    const auto hardware = qobject_cast<BaseHardwareWidget*>(hardwares_->widget(i));
    type_->addItem(hardware->name());
  }

  connect(type_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), this, &self::setCurrentHardware);
  setCurrentHardware(0);
}

void HardwareWidget::onOpened()
{
  return;
}

void HardwareWidget::updateInternalDataStructures()
{
  return;
}

bool HardwareWidget::isValid()
{
  if (!selected()->isValid())
    return false;

  return true;
}

YAML::Node HardwareWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTypeKey] = type_->currentText();

  for (int i = 0; i < hardwares_->count(); ++i)
  {
    const auto hardware = qobject_cast<BaseHardwareWidget*>(hardwares_->widget(i));
    node[hardware->name()] = hardware->dump();
  }

  return node;
}

void HardwareWidget::load(const YAML::Node& node)
{
  type_->setCurrentText(node[kTypeKey].as<QString>());

  for (int i = 0; i < hardwares_->count(); ++i)
  {
    const auto hardware = qobject_cast<BaseHardwareWidget*>(hardwares_->widget(i));
    hardware->load(node[hardware->name()]);
  }
}

const char* HardwareWidget::hardwarePackage() const
{
  return selected()->hardwarePackage();
}

void HardwareWidget::setCurrentHardware(int index)
{
  hardwares_->setCurrentIndex(index);
  description_->setText(selected()->description());
}

BaseHardwareWidget* HardwareWidget::selected()
{
  return qobject_cast<BaseHardwareWidget*>(hardwares_->currentWidget());
}

const BaseHardwareWidget* HardwareWidget::selected() const
{
  return qobject_cast<const BaseHardwareWidget*>(hardwares_->currentWidget());
}
}  // namespace setup_assistant
}  // namespace gui
