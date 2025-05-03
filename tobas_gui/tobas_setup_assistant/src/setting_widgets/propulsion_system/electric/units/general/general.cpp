#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/general/general.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
GeneralWidget::GeneralWidget(const RobotInfo& robot, Signals& _signals, const QString& link_name)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  channel_ = new ParamGetterWidget_SpinBox("Channel", "");
  channel_->setMinimum(0);
  rows->addWidget(channel_);

  direction_ = new ParamGetterWidget_ComboBox("Turning Direction", "");
  direction_->setChoices({ kCWName, kCCWName });
  rows->addWidget(direction_);

  active_tilt_settings_ = new ActiveTiltSettingsWidget(robot, _signals, link_name);
  rows->addWidget(active_tilt_settings_);

  rows->addStretch();
}

const char* GeneralWidget::name() const
{
  return "General";
}

bool GeneralWidget::isValid()
{
  if (!active_tilt_settings_->isValid()) {
    return false;
  }

  return true;
}

void GeneralWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<GeneralWidget>(src);

  channel_->setValue(derived->channel_->getValue());
  direction_->setValue(derived->direction_->getValue());
  active_tilt_settings_->copyFrom(derived->active_tilt_settings_);
}

YAML::Node GeneralWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[channel_->name()] = channel_->getValue();
  node[direction_->name()] = direction_->getValue();
  node[kActiveTiltSettingsKey] = active_tilt_settings_->dump();

  return node;
}

void GeneralWidget::load(const YAML::Node& node)
{
  channel_->setValue(node[channel_->name()].as<int>());
  direction_->setValue(node[direction_->name()].as<QString>());
  active_tilt_settings_->load(node[kActiveTiltSettingsKey]);
}

int GeneralWidget::channel() const
{
  return channel_->getValue();
}

tobas::turning_direction_t GeneralWidget::direction() const
{
  const auto text = direction_->getValue();
  if (text == kCWName) {
    return tobas::turning_direction_t::CW;
  }
  else if (text == kCCWName) {
    return tobas::turning_direction_t::CCW;
  }
  else {
    throw;
  }
}

bool GeneralWidget::isTiltRotor() const
{
  return active_tilt_settings_->isTiltRotor();
}

QString GeneralWidget::tiltJointName() const
{
  return active_tilt_settings_->tiltJointName();
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
