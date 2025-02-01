#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/general/general.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion
{
GeneralWidget::GeneralWidget(const RobotInfo& robot, const QString& link_name)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  channel_ = new ParamGetterWidget_SpinBox("Channel", "");
  channel_->setMinimum(0);
  rows->addWidget(channel_);

  active_tilt_settings_ = new ActiveTiltSettingsWidget(robot, link_name);
  rows->addWidget(active_tilt_settings_);

  rows->addStretch();

  connect(channel_, &ParamGetterWidget_SpinBox::valueChanged, [this](int channel) { Q_EMIT channelChanged(channel); });
  connect(
    active_tilt_settings_, &ActiveTiltSettingsWidget::isTiltStateChanged,
    [this](bool is_tilt) { Q_EMIT isTiltStateChanged(is_tilt); });
  connect(
    active_tilt_settings_, &ActiveTiltSettingsWidget::tiltJointNameChanged,
    [this](const QString& joint_name) { Q_EMIT tiltJointNameChanged(joint_name); });
}

const char* GeneralWidget::name() const
{
  return "General";
}

bool GeneralWidget::isValid()
{
  if (!active_tilt_settings_->isValid())
    return false;

  return true;
}

void GeneralWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qobject_cast<const GeneralWidget*>(src);

  channel_->setValue(derived->channel_->getValue());
  active_tilt_settings_->copyFrom(derived->active_tilt_settings_);
}

YAML::Node GeneralWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[channel_->name()] = channel_->getValue();
  node[kActiveTiltSettingsKey] = active_tilt_settings_->dump();

  return node;
}

void GeneralWidget::load(const YAML::Node& node)
{
  blockSignals(true);

  channel_->setValue(node[channel_->name()].as<int>());
  active_tilt_settings_->load(node[kActiveTiltSettingsKey]);

  blockSignals(false);
}

int GeneralWidget::channel() const
{
  return channel_->getValue();
}

bool GeneralWidget::isTiltRotor() const
{
  return active_tilt_settings_->isTiltRotor();
}

QString GeneralWidget::tiltJointName() const
{
  return active_tilt_settings_->tiltJointName();
}
}  // namespace propulsion
}  // namespace setup_assistant
}  // namespace gui
