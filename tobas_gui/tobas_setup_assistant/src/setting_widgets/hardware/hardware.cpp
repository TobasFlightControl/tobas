#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/hardware/hardware.hpp"
#include "tobas_setup_assistant/setting_tabs/hardware/t1.hpp"

namespace gui
{
namespace sa
{
HardwareWidget::HardwareWidget()
{
  type_ = new qt::ComboBox();
  hardwares_ = new qt::StackedWidget();
  description_ = new qt::DescriptionWidget("", kBodyPSize);

  addWidget(type_);
  addWidget(description_);
  addWidget(hardwares_);

  hardwares_->addWidget(new T1Widget());

  for (int i = 0; i < hardwares_->count(); ++i)
  {
    const auto hardware = qt::qConstPointerCast<BaseHardwareWidget>(hardwares_->widget(i));
    type_->addItem(hardware->name());
  }

  connect(type_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), this, &self::setCurrentHardware);
  setCurrentHardware(0);
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

YAML::Node HardwareWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTypeKey] = type_->currentText();

  for (int i = 0; i < hardwares_->count(); ++i)
  {
    const auto hardware = qt::qConstPointerCast<BaseHardwareWidget>(hardwares_->widget(i));
    node[hardware->name()] = hardware->dump();
  }

  return node;
}

void HardwareWidget::load(const YAML::Node& node)
{
  type_->setCurrentText(node[kTypeKey].as<QString>());

  for (int i = 0; i < hardwares_->count(); ++i)
  {
    const auto hardware = qt::qPointerCast<BaseHardwareWidget>(hardwares_->widget(i));
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
  return qt::qPointerCast<BaseHardwareWidget>(hardwares_->currentWidget());
}

const BaseHardwareWidget* HardwareWidget::selected() const
{
  return qt::qConstPointerCast<BaseHardwareWidget>(hardwares_->currentWidget());
}
}  // namespace sa
}  // namespace gui
