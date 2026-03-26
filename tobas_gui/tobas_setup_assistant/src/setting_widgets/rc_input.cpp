#include "tobas_setup_assistant/setting_tabs/rc_input.hpp"

#include <tobas_constants/rc_input.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
RcInputWidget::RcInputWidget()
{
  num_sbus_channels_ = new ParamGetterWidget_SpinBox("The number of S.BUS channels", "");
  num_sbus_channels_->setMinimum(tobas::kMinSbusChannels);
  num_sbus_channels_->setMaximum(tobas::kMaxSbusChannels);
  num_sbus_channels_->setValue(tobas::kMinSbusChannels);
  addWidget(num_sbus_channels_);

  addStretch();
}

const char* RcInputWidget::name() const
{
  return "RC Input";
}

const char* RcInputWidget::title() const
{
  return "Configure Radio Control";
}

const char* RcInputWidget::description() const
{
  return "Configure the parameters for transmitter-based remote operation. "
         "Tobas supports S.BUS and S.BUS2 receiver protocols. "
         "Enter the appropriate values in each field.";
}

void RcInputWidget::updateInternalDataStructures()
{
}

bool RcInputWidget::isValid()
{
  return true;
}

YAML::Node RcInputWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[num_sbus_channels_->name()] = num_sbus_channels_->getValue();

  return node;
}

void RcInputWidget::load(const YAML::Node& node)
{
  num_sbus_channels_->setValue(node[num_sbus_channels_->name()].as<int>());
}

int RcInputWidget::numOfSbusChannels() const
{
  return num_sbus_channels_->getValue();
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
