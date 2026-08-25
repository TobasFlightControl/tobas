// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/geometry_based/control_surfaces.hpp"

#include <QDebug>
#include <QHeaderView>
#include <QLabel>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_yaml_tools/format.hpp>

#include <tobas_setup_assistant/setting_tabs/fixed_wing/constants.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace gb
{
ControlSurfacesWidget::ControlSurfacesWidget(const uadf::Model& uadf, WingsWidget* wings_widget)
  : super(0, kNumCols), uadf_(uadf), wings_widget_(wings_widget)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  setHorizontalHeaderLabels(
    { kLinkNameLabel, kJointNameLabel, kWingLabel, kStartSpanLabel, kFinishSpanLabel, kChordRatioLabel });
  setColumnsWidth(kColWidth);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  setHeaderSectionsClickable(false);
  horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);

  // connections
  connect(wings_widget_, &WingsWidget::tabAdded, this, &self::onWingsTabAdded);
  connect(wings_widget_, &WingsWidget::tabRemoved, this, &self::onWingsTabRemoved);
  connect(wings_widget_, &WingsWidget::tabRenamed, this, &self::onWingsTabRenamed);
}

void ControlSurfacesWidget::updateInternalDataStructures()
{
  removeAll();

  for (const auto& [joint_name, _] : uadf_.control_surfaces) {
    const auto& link_name = uadf_.urdf->getJoint(joint_name)->child_link_name;
    add(QString::fromStdString(link_name));
  }

  setFixedHeight(horizontalHeader()->height() + verticalHeader()->length() + 2 * frameWidth());

  updateGeometry();
}

void ControlSurfacesWidget::setToDefaults()
{
  for (int row = 0; row < rowCount(); ++row) {
    setToDefault(row);
  }
}

bool ControlSurfacesWidget::isValid()
{
  return true;
}

YAML::Node ControlSurfacesWidget::dump() const
{
  YAML::Node node;

  for (int row = 0; row < rowCount(); ++row) {
    YAML::Node sub_node(YAML::NodeType::Map);
    sub_node[kJointNameLabel] = jointName(row);
    sub_node[kWingLabel] = wingIdx(row);
    sub_node[kStartSpanLabel] = yaml::format(startSpan(row));
    sub_node[kFinishSpanLabel] = yaml::format(finishSpan(row));
    sub_node[kChordRatioLabel] = yaml::format(chordRatio(row));

    const auto link_name = linkName(row);
    node[link_name.toStdString()] = sub_node;
  }

  return node;
}

void ControlSurfacesWidget::load(const YAML::Node& node)
{
  for (const auto& pair : node) {
    const auto link_name = pair.first.as<QString>();
    const auto& sub_node = pair.second;

    const auto row = find(link_name);
    if (row < 0) {
      throw std::runtime_error("Failed to find CS link \"" + link_name.toStdString() + "\".");
    }

    linkName(row, link_name);
    jointName(row, sub_node[kJointNameLabel].as<QString>());
    wingIdx(row, sub_node[kWingLabel].as<int>());
    startSpan(row, sub_node[kStartSpanLabel].as<double>());
    finishSpan(row, sub_node[kFinishSpanLabel].as<double>());
    chordRatio(row, sub_node[kChordRatioLabel].as<double>());
  }
}

void ControlSurfacesWidget::add(const QString& link_name)
{
  const auto joint = uadf_.urdf->getLink(link_name.toStdString())->parent_joint;

  const auto row = rowCount();
  insertRow(row);

  const auto link_name_label = new QLabel(link_name);
  link_name_label->setFont(qt::DefaultFont(cmn::kBodyPSize));
  link_name_label->setAlignment(Qt::AlignCenter);
  setCellWidget(row, kLinkNameCol, link_name_label);

  const auto joint_name_label = new QLabel(QString::fromStdString(joint->name));
  joint_name_label->setFont(qt::DefaultFont(cmn::kBodyPSize));
  joint_name_label->setAlignment(Qt::AlignCenter);
  setCellWidget(row, kJointNameCol, joint_name_label);

  const auto wing = new qt::ComboBox();
  wing->insertItem(0, wings_widget_->requiredWingName());
  setCellWidget(row, kWingCol, wing);

  const auto start_span = new qt::DoubleSpinBox();
  start_span->setDecimals(2);
  start_span->setSuffix("");
  start_span->setMinimum(-1.0);
  start_span->setMaximum(1.0);
  setCellWidget(row, kStartSpanCol, start_span);

  const auto finish_span = new qt::DoubleSpinBox();
  finish_span->setDecimals(2);
  finish_span->setSuffix("");
  finish_span->setMinimum(-1.0);
  finish_span->setMaximum(1.0);
  setCellWidget(row, kFinishSpanCol, finish_span);

  const auto chord_ratio = new qt::DoubleSpinBox();
  chord_ratio->setDecimals(2);
  chord_ratio->setSuffix("");
  chord_ratio->setMinimum(0.01);
  chord_ratio->setMaximum(1.0);
  setCellWidget(row, kChordRatioCol, chord_ratio);

  setToDefault(row);
}

QString ControlSurfacesWidget::linkName(int row) const
{
  const auto cell = qt::qConstPointerCast<QLabel>(cellWidget(row, kLinkNameCol));
  return cell->text();
}

QString ControlSurfacesWidget::jointName(int row) const
{
  const auto cell = qt::qConstPointerCast<QLabel>(cellWidget(row, kJointNameCol));
  return cell->text();
}

int ControlSurfacesWidget::wingIdx(int row) const
{
  const auto cell = qt::qConstPointerCast<qt::ComboBox>(cellWidget(row, kWingCol));
  return cell->currentIndex();
}

double ControlSurfacesWidget::startSpan(int row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(row, kStartSpanCol));
  return cell->value();
}

double ControlSurfacesWidget::finishSpan(int row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(row, kFinishSpanCol));
  return cell->value();
}

double ControlSurfacesWidget::chordRatio(int row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(row, kChordRatioCol));
  return cell->value();
}

void ControlSurfacesWidget::linkName(int row, const QString& text)
{
  const auto cell = qt::qPointerCast<QLabel>(cellWidget(row, kLinkNameCol));
  return cell->setText(text);
}

void ControlSurfacesWidget::jointName(int row, const QString& text)
{
  const auto cell = qt::qPointerCast<QLabel>(cellWidget(row, kJointNameCol));
  return cell->setText(text);
}

void ControlSurfacesWidget::wingIdx(int row, int index)
{
  const auto cell = qt::qPointerCast<qt::ComboBox>(cellWidget(row, kWingCol));
  return cell->setCurrentIndex(index);
}

void ControlSurfacesWidget::startSpan(int row, double value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(row, kStartSpanCol));
  return cell->setValue(value);
}

void ControlSurfacesWidget::finishSpan(int row, double value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(row, kFinishSpanCol));
  return cell->setValue(value);
}

void ControlSurfacesWidget::chordRatio(int row, double value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(row, kChordRatioCol));
  return cell->setValue(value);
}

void ControlSurfacesWidget::setToDefault(int row)
{
  wingIdx(row, 0);
  startSpan(row, -1.0);
  finishSpan(row, 1.0);
  chordRatio(row, 0.2);
}

int ControlSurfacesWidget::find(const QString& link_name) const
{
  for (int row = 0; row < rowCount(); ++row) {
    if (linkName(row) == link_name) {
      return row;
    }
  }

  qWarning() << link_name << "is not selected as a control surface.";
  return -1;
}

void ControlSurfacesWidget::onWingsTabAdded(QString tab_name)
{
  for (int row = 0; row < rowCount(); row++) {
    const auto cell = qt::qPointerCast<qt::ComboBox>(cellWidget(row, kWingCol));
    cell->addItem(tab_name);
  }
}

void ControlSurfacesWidget::onWingsTabRemoved(int index)
{
  for (int row = 0; row < rowCount(); row++) {
    const auto cell = qt::qPointerCast<qt::ComboBox>(cellWidget(row, kWingCol));
    cell->removeItem(index);
  }
}

void ControlSurfacesWidget::onWingsTabRenamed(int index, QString new_name)
{
  for (int row = 0; row < rowCount(); row++) {
    const auto cell = qt::qPointerCast<qt::ComboBox>(cellWidget(row, kWingCol));
    cell->setItemText(index, new_name);
  }
}

}  // namespace gb
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
