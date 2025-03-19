#include <QLabel>
#include <QVBoxLayout>
#include <QFormLayout>

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/common.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/general/active_tilt_settings.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
ActiveTiltSettingsWidget::ActiveTiltSettingsWidget(const RobotInfo& robot, const QString& link_name)
  : robot_(robot), link_name_(link_name)
{
  const auto label = new qt::Label("Active Tilt Settings", kLabelPSize, QFont::Bold);

  is_tilt_ = new QCheckBox("Use as an active tilt rotor");
  is_tilt_->setChecked(false);

  tilt_joint_name_ = new QComboBox();
  tilt_joint_name_->setEnabled(false);

  // Layout
  const auto form = new QFormLayout();
  form->addRow("Tilt Joint", tilt_joint_name_);

  const auto rows = new QVBoxLayout();
  rows->addWidget(label);
  rows->addWidget(is_tilt_);
  rows->addLayout(form);

  setLayout(rows);

  // Connection
  connect(is_tilt_, &QCheckBox::toggled, this, &self::onIsTiltCheckBoxToggled);
  connect(tilt_joint_name_, &QComboBox::currentTextChanged, this, &self::onTiltJointNameChanged);
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
  blockSignals(true);

  is_tilt_->setChecked(node[kIsTiltKey].as<bool>());
  tilt_joint_name_->setCurrentText(node[kTiltJointNameKey].as<QString>());

  blockSignals(false);
}

bool ActiveTiltSettingsWidget::isTiltRotor() const
{
  return is_tilt_->isChecked();
}

QString ActiveTiltSettingsWidget::tiltJointName() const
{
  return tilt_joint_name_->currentText();
}

void ActiveTiltSettingsWidget::onIsTiltCheckBoxToggled(bool checked)
{
  if (checked)
  {
    auto seg_it = robot_.tree().getSegment(link_name_.toStdString());
    seg_it = seg_it->second.parent;

    // ルートリンクまでの全ての回転関節を選択肢に追加
    while (seg_it != robot_.tree().getRootSegment())
    {
      const auto& elem = seg_it->second;
      const auto& joint = elem.segment.joint();
      if (joint.type == kdl::Joint::ROTATION)
        tilt_joint_name_->addItem(QString::fromStdString(joint.name));
      seg_it = elem.parent;
    }

    // ティルトジョイントの候補が存在しなければリセット
    if (tilt_joint_name_->count() == 0)
    {
      qt::qWarnBox(this, "\"" + link_name_ + "\" cannot be used as a tilt rotor.");

      // チェック状態をリセットする際にシグナルが発行されないようブロック
      is_tilt_->blockSignals(true);
      is_tilt_->setChecked(false);
      is_tilt_->blockSignals(false);

      return;
    }

    tilt_joint_name_->setEnabled(true);
  }
  else
  {
    tilt_joint_name_->clear();
    tilt_joint_name_->setEnabled(false);
  }

  Q_EMIT isTiltStateChanged(checked);
}

void ActiveTiltSettingsWidget::onTiltJointNameChanged(const QString& joint_name)
{
  if (!joint_name.isEmpty())
    Q_EMIT tiltJointNameChanged(joint_name);
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
