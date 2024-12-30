#include <QLabel>
#include <QVBoxLayout>
#include <QFormLayout>

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_setup_assistant/common.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/general/active_tilt_settings.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
ActiveTiltSettingsWidget::ActiveTiltSettingsWidget()
{
  const auto label = new qt::Label("Active Tilt Settings", kLabelPSize, QFont::Bold);

  is_tilt_ = new QCheckBox("Use as an active tilt rotor");
  is_tilt_->setChecked(false);

  tilt_joint_name_ = new QComboBox();

  // Layout
  const auto form = new QFormLayout();
  form->addRow("Tilt Joint", tilt_joint_name_);

  const auto rows = new QVBoxLayout();
  rows->addWidget(label);
  rows->addWidget(is_tilt_);
  rows->addLayout(form);

  setLayout(rows);
}

bool ActiveTiltSettingsWidget::isValid()
{
  return true;
}

void ActiveTiltSettingsWidget::copyFrom(const ActiveTiltSettingsWidget* src)
{
  is_tilt_->setChecked(src->is_tilt_->isChecked());
  tilt_joint_name_->setCurrentText(src->tilt_joint_name_->currentText());
}

YAML::Node ActiveTiltSettingsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kIsTiltKey] = is_tilt_->isChecked();
  node[kTiltJointNameKey] = tilt_joint_name_->currentText();

  return node;
}

void ActiveTiltSettingsWidget::load(const YAML::Node& node)
{
  is_tilt_->setChecked(node[kIsTiltKey].as<bool>());
  tilt_joint_name_->setCurrentText(node[kTiltJointNameKey].as<QString>());
}

bool ActiveTiltSettingsWidget::isTiltRotor() const
{
  return is_tilt_->isChecked();
}

QString ActiveTiltSettingsWidget::tiltJointName() const
{
  return tilt_joint_name_->currentText();
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
