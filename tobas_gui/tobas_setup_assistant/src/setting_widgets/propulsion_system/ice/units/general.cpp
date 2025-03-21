#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/general.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
GeneralWidget::GeneralWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  channel_ = new ParamGetterWidget_SpinBox("Channel", "");
  channel_->setMinimum(0);
  rows->addWidget(channel_);

  direction_ = new ParamGetterWidget_ComboBox("Turning Direction", "");
  direction_->setChoices({ kCWName, kCCWName });
  rows->addWidget(direction_);

  rows->addStretch();
}

const char* GeneralWidget::name() const
{
  return "General";
}

bool GeneralWidget::isValid()
{
  return true;
}

void GeneralWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<GeneralWidget>(src);

  channel_->setValue(derived->channel_->getValue());
  direction_->setValue(derived->direction_->getValue());
}

YAML::Node GeneralWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[channel_->name()] = channel_->getValue();
  node[direction_->name()] = direction_->getValue();

  return node;
}

void GeneralWidget::load(const YAML::Node& node)
{
  channel_->setValue(node[channel_->name()].as<int>());
  direction_->setValue(node[direction_->name()].as<QString>());
}

int GeneralWidget::channel() const
{
  return channel_->getValue();
}

tobas::turning_direction_t GeneralWidget::direction() const
{
  const auto text = direction_->getValue();
  if (text == kCWName)
    return tobas::turning_direction_t::CW;
  else if (text == kCCWName)
    return tobas::turning_direction_t::CCW;
  else
    throw;
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
