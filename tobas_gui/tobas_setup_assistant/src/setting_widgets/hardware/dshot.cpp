// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/hardware/dshot.hpp"

#include <ranges>

#include <QDebug>
#include <QHeaderView>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

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
constexpr int kBidirectionalCol = kTargetNameCol + 1;
constexpr int kNumCols = kBidirectionalCol + 1;

constexpr char kTargetNameLabel[] = "Target";
constexpr char kBidirectionalLabel[] = "Bidirectional";
}  // namespace

DShotWidget::DShotWidget(const uadf::Model& _uadf, const Signals& _sig) : super(0, kNumCols), uadf_(_uadf)
{
  setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  setHorizontalHeaderLabels({ kTargetNameLabel, kBidirectionalLabel });
  setHeaderSectionsClickable(false);

  const auto hor_header = horizontalHeader();
  hor_header->setSectionResizeMode(QHeaderView::ResizeToContents);
  hor_header->setMinimumSectionSize(kTableHeaderSectionSize);

  connect(&_sig, &Signals::propulsionTypeChanged, this, &self::onPropulsionTypeChanged);
}

void DShotWidget::updateInternalDataStructures()
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

bool DShotWidget::isValid()
{
  // Confirm that target names are not duplicated.
  QSet<QString> target_name_set;
  for (int channel = 0; channel < rowCount(); ++channel) {
    const auto target_name = targetName(channel);
    if (target_name.isEmpty()) {
      continue;
    }
    if (target_name_set.contains(target_name)) {
      qt::qWarnBox(this, "DShot target '" + target_name + "' is duplicated.");
      return false;
    }
    target_name_set.insert(target_name);
  }

  return true;
}

YAML::Node DShotWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Sequence);

  for (int channel = 0; channel < rowCount(); ++channel) {
    YAML::Node sub_node(YAML::NodeType::Map);

    sub_node[kTargetNameLabel] = targetNameWidget(channel)->currentText();
    sub_node[kBidirectionalLabel] = bidirectionalWidget(channel)->isChecked();

    node.push_back(sub_node);
  }

  return node;
}

void DShotWidget::load(const YAML::Node& _node)
{
  for (const auto& [channel, sub_node] : std::views::enumerate(_node)) {
    targetNameWidget(channel)->setCurrentText(sub_node[kTargetNameLabel].as<QString>());
    bidirectionalWidget(channel)->setChecked(sub_node[kBidirectionalLabel].as<bool>());
  }
}

void DShotWidget::setNumChannels(int _num)
{
  while (true) {
    if (_num > rowCount()) {
      addLastChannel();
    }
    else if (_num < rowCount()) {
      removeLastChannel();
    }
    else {
      return;
    }
  }
}

QString DShotWidget::targetName(int _channel) const
{
  return targetNameWidget(_channel)->currentText();
}

bool DShotWidget::bidirectional(int _channel) const
{
  return bidirectionalWidget(_channel)->isChecked();
}

bool DShotWidget::contains(const QString& _target_name) const
{
  for (int channel = 0; channel < rowCount(); ++channel) {
    if (targetName(channel) == _target_name) {
      return true;
    }
  }

  return false;
}

int DShotWidget::channel(const QString& _target_name) const
{
  for (int channel = 0; channel < rowCount(); ++channel) {
    if (targetName(channel) == _target_name) {
      return channel;
    }
  }

  qWarning() << _target_name << "not found.";
  return -1;
}

qt::ComboBox* DShotWidget::targetNameWidget(int _row)
{
  return qt::qPointerCast<qt::ComboBox>(cellWidget(_row, kTargetNameCol));
}

QPushButton* DShotWidget::bidirectionalWidget(int _row)
{
  return qt::qPointerCast<QPushButton>(cellWidget(_row, kBidirectionalCol));
}

const qt::ComboBox* DShotWidget::targetNameWidget(int _row) const
{
  return qt::qConstPointerCast<qt::ComboBox>(cellWidget(_row, kTargetNameCol));
}

const QPushButton* DShotWidget::bidirectionalWidget(int _row) const
{
  return qt::qConstPointerCast<QPushButton>(cellWidget(_row, kBidirectionalCol));
}

void DShotWidget::addLastChannel()
{
  const auto row = rowCount();

  // Target name
  const auto target_name = new qt::ComboBox();
  target_name->addItem("");  // Not selected.
  switch (prop_type_) {
    case PropulsionSystem::kElectric: {
      for (const auto& [joint_name, _] : uadf_.thrusts) {
        target_name->addItem(QString::fromStdString(joint_name));
      }
      break;
    }
    case PropulsionSystem::kIce: {
      break;
    }
    default:
      throw;
  }

  // Bidirectional
  const auto bidirectional = new QPushButton();
  bidirectional->setCheckable(true);
  setBidirectionalButtonChecked(bidirectional, true);  // Bidirectional communication by default.
  connect(
    bidirectional,
    &QPushButton::toggled,
    std::bind(&self::onBidirectionalButtonToggled, this, bidirectional, std::placeholders::_1));
  bidirectional->setEnabled(false);  // TODO: Support unidirectional communication.

  // Insert table row.
  insertRow(row);
  setVerticalHeaderItem(row, new QTableWidgetItem("CH" + QString::number(row)));
  setCellWidget(row, kTargetNameCol, target_name);
  setCellWidget(row, kBidirectionalCol, bidirectional);
}

void DShotWidget::removeLastChannel()
{
  const auto row = rowCount() - 1;

  if (row < 0) {
    return;
  }

  const auto target_name = targetName(row);

  removeRow(row);

  if (!target_name.isEmpty()) {
    qt::qWarnBox(this, "PWM configuration for '" + target_name + "' has been removed.");
  }
}

void DShotWidget::setBidirectionalButtonChecked(QPushButton* _button, bool _checked)
{
  _button->setChecked(_checked);
  setBidirectionalButtonText(_button, _checked);
}

void DShotWidget::setBidirectionalButtonText(QPushButton* _button, bool _checked)
{
  if (_checked) {
    _button->setText("Enabled");
  }
  else {
    _button->setText("Disabled");
  }
}

void DShotWidget::onPropulsionTypeChanged(PropulsionSystem _new_prop_type)
{
  if (_new_prop_type == prop_type_) {
    return;
  }

  // Remove unnecessary choices from the previous propulsion system.
  switch (prop_type_) {
    case PropulsionSystem::kElectric: {
      for (int channel = 0; channel < rowCount(); ++channel) {
        const auto target_name = targetNameWidget(channel);

        for (const auto& [joint_name, _] : uadf_.thrusts) {
          if (target_name->currentText().toStdString() == joint_name) {
            target_name->setCurrentText("");
          }
          target_name->removeText(QString::fromStdString(joint_name));
        }
      }
      break;
    }
    case PropulsionSystem::kIce: {
      break;
    }
    default:
      throw;
  }

  // Add choices for the new propulsion system.
  switch (_new_prop_type) {
    case PropulsionSystem::kElectric: {
      for (int channel = 0; channel < rowCount(); ++channel) {
        const auto target_name = targetNameWidget(channel);

        for (const auto& [joint_name, _] : uadf_.thrusts) {
          target_name->addItem(QString::fromStdString(joint_name));
        }
      }
      break;
    }
    case PropulsionSystem::kIce: {
      break;
    }
    default:
      throw;
  }

  prop_type_ = _new_prop_type;
}

void DShotWidget::onBidirectionalButtonToggled(QPushButton* _button, bool _checked)
{
  setBidirectionalButtonText(_button, _checked);
}
}  // namespace hw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
