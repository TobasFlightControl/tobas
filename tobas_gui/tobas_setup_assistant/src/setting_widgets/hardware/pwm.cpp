#include "tobas_setup_assistant/setting_tabs/hardware/pwm.hpp"

#include <ranges>

#include <QDebug>
#include <QHeaderView>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/hardware/constants.hpp"

namespace gui
{
namespace sa
{
namespace hw
{
PwmWidget::PwmWidget(const RobotInfo& robot, const Signals& sig) : super(0, kNumCols), robot_(robot)
{
  horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  horizontalHeader()->setMinimumSectionSize(kTableHeaderSectionSize);
  setHorizontalHeaderLabels({ kTargetNameLabel, kPeriodLbLabel, kPeriodUbLabel });

  connect(&sig, &Signals::propulsionTypeChanged, this, &self::onPropulsionTypeChanged);
}

void PwmWidget::updateInternalDataStructures()
{
  // 現在の行数を保存
  const auto rows = rowCount();

  // 設定をリセットするために一旦全削除
  removeAll();

  // 更新された選択肢でチャンネルを加え直す
  for (int _ = 0; _ < rows; ++_) {
    addLastChannel();
  }
}

bool PwmWidget::isValid()
{
  // ターゲット名が重複していないことを確認
  QSet<QString> target_name_set;
  for (int channel = 0; channel < rowCount(); ++channel) {
    const auto target_name = targetName(channel);
    if (target_name.isEmpty()) {
      continue;
    }
    if (target_name_set.contains(target_name)) {
      qt::qErrorBox(this, "PWM target \"" + target_name + "\" is duplicated.");
      return false;
    }
    target_name_set.insert(target_name);
  }

  return true;
}

YAML::Node PwmWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Sequence);

  for (int channel = 0; channel < rowCount(); ++channel) {
    YAML::Node sub_node(YAML::NodeType::Map);

    sub_node[kTargetNameLabel] = targetNameWidget(channel)->currentText();
    sub_node[kPeriodLbLabel] = periodLbWidget(channel)->value();
    sub_node[kPeriodUbLabel] = periodUbWidget(channel)->value();

    node.push_back(sub_node);
  }

  return node;
}

void PwmWidget::load(const YAML::Node& node)
{
  for (const auto& [channel, sub_node] : std::views::enumerate(node)) {
    targetNameWidget(channel)->setCurrentText(sub_node[kTargetNameLabel].as<QString>());
    periodLbWidget(channel)->setValue(sub_node[kPeriodLbLabel].as<int>());
    periodUbWidget(channel)->setValue(sub_node[kPeriodUbLabel].as<int>());
  }
}

void PwmWidget::setNumChannels(int num)
{
  while (true) {
    if (num > rowCount()) {
      addLastChannel();
    }
    else if (num < rowCount()) {
      removeLastChannel();
    }
    else {
      return;
    }
  }
}

QString PwmWidget::targetName(int channel) const
{
  return targetNameWidget(channel)->currentText();
}

PwmWidget::TargetType PwmWidget::targetType(int channel) const
{
  const auto target_name = targetName(channel).toStdString();

  if (robot_.uadf().thrusts.contains(target_name)) {
    return TargetType::kThrust;
  }
  else if (robot_.uadf().control_surfaces.contains(target_name)) {
    return TargetType::kControlSurface;
  }
  else if (robot_.uadf().tilts.contains(target_name)) {
    return TargetType::kTiltJoint;
  }
  else if (target_name == kEngineThrotLabel) {
    return TargetType::kEngineThrottle;
  }
  else {
    throw std::runtime_error("Invalid PWM target_name name: " + target_name);
  }
}

uint16_t PwmWidget::periodLb(int channel) const
{
  return periodLbWidget(channel)->value();
}

uint16_t PwmWidget::periodUb(int channel) const
{
  return periodUbWidget(channel)->value();
}

bool PwmWidget::contains(const QString& target_name) const
{
  for (int channel = 0; channel < rowCount(); ++channel) {
    if (targetName(channel) == target_name) {
      return true;
    }
  }

  return false;
}

int PwmWidget::channel(const QString& target_name) const
{
  for (int channel = 0; channel < rowCount(); ++channel) {
    if (targetName(channel) == target_name) {
      return channel;
    }
  }

  qWarning() << "Failed to find \"" << target_name << "\".";
  return -1;
}

qt::ComboBox* PwmWidget::targetNameWidget(int row)
{
  return qt::qPointerCast<qt::ComboBox>(cellWidget(row, kTargetNameCol));
}

qt::SpinBox* PwmWidget::periodLbWidget(int row)
{
  return qt::qPointerCast<qt::SpinBox>(cellWidget(row, kPeriodLbCol));
}

qt::SpinBox* PwmWidget::periodUbWidget(int row)
{
  return qt::qPointerCast<qt::SpinBox>(cellWidget(row, kPeriodUbCol));
}

const qt::ComboBox* PwmWidget::targetNameWidget(int row) const
{
  return qt::qConstPointerCast<qt::ComboBox>(cellWidget(row, kTargetNameCol));
}

const qt::SpinBox* PwmWidget::periodLbWidget(int row) const
{
  return qt::qConstPointerCast<qt::SpinBox>(cellWidget(row, kPeriodLbCol));
}

const qt::SpinBox* PwmWidget::periodUbWidget(int row) const
{
  return qt::qConstPointerCast<qt::SpinBox>(cellWidget(row, kPeriodUbCol));
}

void PwmWidget::addLastChannel()
{
  const auto row = rowCount();

  // Target name
  const auto target_name = new qt::ComboBox();
  target_name->addItem("");  // 未選択
  for (const auto& [joint_name, _] : robot_.uadf().tilts) {
    target_name->addItem(QString::fromStdString(joint_name));
  }
  for (const auto& [joint_name, _] : robot_.uadf().control_surfaces) {
    target_name->addItem(QString::fromStdString(joint_name));
  }
  switch (prop_type_) {
    case tobas::propulsion_system_t::ELECTRIC: {
      break;
    }
    case tobas::propulsion_system_t::ICE: {
      for (const auto& [joint_name, _] : robot_.uadf().thrusts) {
        target_name->addItem(QString::fromStdString(joint_name));
      }
      target_name->addItem(kEngineThrotLabel);
      break;
    }
    default:
      throw;
  }

  // PWM period (LB)
  const auto period_lb = new qt::SpinBox();
  period_lb->setMinimum(1);
  period_lb->setMaximum(2499);
  period_lb->setValue(1000);
  period_lb->setSuffix(" us");

  // PWM period (UB)
  const auto period_ub = new qt::SpinBox();
  period_ub->setMinimum(1);
  period_ub->setMaximum(2499);
  period_ub->setValue(2000);
  period_ub->setSuffix(" us");

  // Insert table row
  insertRow(row);
  setVerticalHeaderItem(row, new QTableWidgetItem("CH" + QString::number(row)));
  setCellWidget(row, kTargetNameCol, target_name);
  setCellWidget(row, kPeriodLbCol, period_lb);
  setCellWidget(row, kPeriodUbCol, period_ub);
}

void PwmWidget::removeLastChannel()
{
  const auto row = rowCount() - 1;

  if (row < 0) {
    return;
  }

  const auto target_name = targetName(row);

  removeRow(row);

  if (!target_name.isEmpty()) {
    qt::qWarnBox(this, "PWM configuration for \"" + target_name + "\" has been removed.");
  }
}

void PwmWidget::onPropulsionTypeChanged(const tobas::propulsion_system_t& new_prop_type)
{
  if (new_prop_type == prop_type_) {
    return;
  }

  // 前の推進系の不要な選択肢を外す
  switch (prop_type_) {
    case tobas::propulsion_system_t::ELECTRIC: {
      break;
    }
    case tobas::propulsion_system_t::ICE: {
      for (int channel = 0; channel < rowCount(); ++channel) {
        const auto target_name = targetNameWidget(channel);

        for (const auto& [joint_name, _] : robot_.uadf().thrusts) {
          if (target_name->currentText().toStdString() == joint_name) {
            target_name->setCurrentText("");
          }
          target_name->removeText(QString::fromStdString(joint_name));
        }

        if (target_name->currentText() == kEngineThrotLabel) {
          target_name->setCurrentText("");
        }
        target_name->removeText(kEngineThrotLabel);
      }

      break;
    }
    default:
      throw;
  }

  // 新しい推進系の選択肢を追加
  switch (prop_type_) {
    case tobas::propulsion_system_t::ELECTRIC: {
      break;
    }
    case tobas::propulsion_system_t::ICE: {
      for (int channel = 0; channel < rowCount(); ++channel) {
        const auto target_name = targetNameWidget(channel);

        for (const auto& [joint_name, _] : robot_.uadf().thrusts) {
          target_name->addItem(QString::fromStdString(joint_name));
        }

        target_name->addItem(kEngineThrotLabel);
      }

      break;
    }
    default:
      throw;
  }

  prop_type_ = new_prop_type;
}
}  // namespace hw
}  // namespace sa
}  // namespace gui
