#include <tobas_constants/constants.hpp>

#include "tobas_setup_assistant/setting_tabs/rc_input.hpp"

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
  return "";  // TODO
}

void RcInputWidget::onOpened()
{
  return;
}

void RcInputWidget::updateInternalDataStructures()
{
  return;
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
