// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/hardware/pwm.hpp"

#include <ranges>

#include <QDebug>
#include <QHeaderView>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_yaml_tools/format.hpp>

#include "tobas_setup_assistant/setting_tabs/hardware/constants.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace hw
{
namespace
{
constexpr int kTargetNameCol = 0;
constexpr int kPeriodLbCol = kTargetNameCol + 1;
constexpr int kPeriodUbCol = kPeriodLbCol + 1;
constexpr int kNumCols = kPeriodUbCol + 1;

constexpr char kTargetNameLabel[] = "Target";
constexpr char kPeriodLbLabel[] = "PWM Period (LB)";
constexpr char kPeriodUbLabel[] = "PWM Period (UB)";

constexpr int kPeriodDecimals = 2;
}  // namespace

PwmWidget::PwmWidget(const uadf::Model& uadf, const Signals& sig) : super(0, kNumCols), uadf_(uadf)
{
  setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  setHorizontalHeaderLabels({ kTargetNameLabel, kPeriodLbLabel, kPeriodUbLabel });
  setHeaderSectionsClickable(false);

  const auto hor_header = horizontalHeader();
  hor_header->setSectionResizeMode(QHeaderView::ResizeToContents);
  hor_header->setMinimumSectionSize(kTableHeaderSectionSize);

  connect(&sig, &Signals::propulsionTypeChanged, this, &self::onPropulsionTypeChanged);
}

void PwmWidget::updateInternalDataStructures()
{
  // Save the current row count.
  const auto rows = rowCount();

  // Delete everything once to reset settings.
  removeAll();

  // Add channels again with the updated choices.
  for (int _ = 0; _ < rows; ++_) {
    addLastChannel();
  }
}

bool PwmWidget::isValid()
{
  // Confirm that target names are not duplicated.
  QSet<QString> target_name_set;
  for (int channel = 0; channel < rowCount(); ++channel) {
    const auto target_name = targetName(channel);
    if (target_name.isEmpty()) {
      continue;
    }
    if (target_name_set.contains(target_name)) {
      qt::qWarnBox(this, "PWM target \"" + target_name + "\" is duplicated.");
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
    sub_node[kPeriodLbLabel] = yaml::format(periodLbWidget(channel)->value());
    sub_node[kPeriodUbLabel] = yaml::format(periodUbWidget(channel)->value());

    node.push_back(sub_node);
  }

  return node;
}

void PwmWidget::load(const YAML::Node& node)
{
  for (const auto& [channel, sub_node] : std::views::enumerate(node)) {
    targetNameWidget(channel)->setCurrentText(sub_node[kTargetNameLabel].as<QString>());
    periodLbWidget(channel)->setValue(sub_node[kPeriodLbLabel].as<double>());
    periodUbWidget(channel)->setValue(sub_node[kPeriodUbLabel].as<double>());
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

  if (uadf_.thrusts.contains(target_name)) {
    return TargetType::kThrust;
  }
  else if (uadf_.control_surfaces.contains(target_name)) {
    return TargetType::kControlSurface;
  }
  else if (uadf_.tilts.contains(target_name)) {
    return TargetType::kTiltJoint;
  }
  else if (target_name == kEngineThrotLabel) {
    return TargetType::kEngineThrottle;
  }
  else {
    throw std::runtime_error("Invalid PWM target_name name: " + target_name);
  }
}

double PwmWidget::periodLb(int channel) const
{
  return periodLbWidget(channel)->value();
}

double PwmWidget::periodUb(int channel) const
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

  qWarning() << target_name << "not found.";
  return -1;
}

qt::ComboBox* PwmWidget::targetNameWidget(int row)
{
  return qt::qPointerCast<qt::ComboBox>(cellWidget(row, kTargetNameCol));
}

qt::DoubleSpinBox* PwmWidget::periodLbWidget(int row)
{
  return qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(row, kPeriodLbCol));
}

qt::DoubleSpinBox* PwmWidget::periodUbWidget(int row)
{
  return qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(row, kPeriodUbCol));
}

const qt::ComboBox* PwmWidget::targetNameWidget(int row) const
{
  return qt::qConstPointerCast<qt::ComboBox>(cellWidget(row, kTargetNameCol));
}

const qt::DoubleSpinBox* PwmWidget::periodLbWidget(int row) const
{
  return qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(row, kPeriodLbCol));
}

const qt::DoubleSpinBox* PwmWidget::periodUbWidget(int row) const
{
  return qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(row, kPeriodUbCol));
}

void PwmWidget::addLastChannel()
{
  const auto row = rowCount();

  // Target name
  const auto target_name = new qt::ComboBox();
  target_name->addItem("");  // Not selected.
  for (const auto& [joint_name, _] : uadf_.tilts) {
    target_name->addItem(QString::fromStdString(joint_name));
  }
  for (const auto& [joint_name, _] : uadf_.control_surfaces) {
    target_name->addItem(QString::fromStdString(joint_name));
  }
  switch (prop_type_) {
    case PropulsionSystem::kElectric: {
      break;
    }
    case PropulsionSystem::kIce: {
      for (const auto& [joint_name, _] : uadf_.thrusts) {
        target_name->addItem(QString::fromStdString(joint_name));
      }
      target_name->addItem(kEngineThrotLabel);
      break;
    }
    default:
      throw;
  }

  // PWM period (LB)
  const auto period_lb = new qt::DoubleSpinBox();
  period_lb->setDecimals(kPeriodDecimals);
  period_lb->setMinimum(0);
  period_lb->setMaximum(2500);
  period_lb->setValue(1000);
  period_lb->setSuffix(" us");

  // PWM period (UB)
  const auto period_ub = new qt::DoubleSpinBox();
  period_ub->setDecimals(kPeriodDecimals);
  period_ub->setMinimum(0);
  period_ub->setMaximum(2500);
  period_ub->setValue(2000);
  period_ub->setSuffix(" us");

  // Insert table row.
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

void PwmWidget::onPropulsionTypeChanged(const PropulsionSystem& new_prop_type)
{
  if (new_prop_type == prop_type_) {
    return;
  }

  // Remove unnecessary choices from the previous propulsion system.
  switch (prop_type_) {
    case PropulsionSystem::kElectric: {
      break;
    }
    case PropulsionSystem::kIce: {
      for (int channel = 0; channel < rowCount(); ++channel) {
        const auto target_name = targetNameWidget(channel);

        for (const auto& [joint_name, _] : uadf_.thrusts) {
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

  // Add choices for the new propulsion system.
  switch (new_prop_type) {
    case PropulsionSystem::kElectric: {
      break;
    }
    case PropulsionSystem::kIce: {
      for (int channel = 0; channel < rowCount(); ++channel) {
        const auto target_name = targetNameWidget(channel);

        for (const auto& [joint_name, _] : uadf_.thrusts) {
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
}  // namespace tobas
