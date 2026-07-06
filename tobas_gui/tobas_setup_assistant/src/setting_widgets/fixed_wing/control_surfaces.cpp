// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/control_surfaces.hpp"

#include <QDebug>
#include <QHeaderView>
#include <QLabel>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_yaml_tools/format.hpp>

#include "tobas_setup_assistant/setting_tabs/fixed_wing/constants.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
ControlSurfacesWidget::ControlSurfacesWidget(const uadf::Model& uadf) : super(0, kNumCols), uadf_(uadf)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  setHorizontalHeaderLabels({
    kLinkNameLabel,
    kJointNameLabel,
    kLiftCoefLabel,
    kDragCoefLabel,
    kSideCoefLabel,
    kRollCoefLabel,
    kPitchCoefLabel,
    kYawCoefLabel,
  });
  setColumnsWidth(kColWidth);
  setHeaderSectionsClickable(false);
  horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
}

void ControlSurfacesWidget::updateInternalDataStructures()
{
  removeAll();

  for (const auto& [joint_name, _] : uadf_.control_surfaces) {
    const auto& link_name = uadf_.urdf->getJoint(joint_name)->child_link_name;
    add(QString::fromStdString(link_name));
  }
}

bool ControlSurfacesWidget::isValid()
{
  return true;
}

YAML::Node ControlSurfacesWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int row = 0; row < rowCount(); ++row) {
    YAML::Node sub_node(YAML::NodeType::Map);
    sub_node[kJointNameLabel] = jointName(row);
    sub_node[kLiftCoefLabel] = yaml::format(liftCoef(row));
    sub_node[kDragCoefLabel] = yaml::format(dragCoef(row));
    sub_node[kSideCoefLabel] = yaml::format(sideCoef(row));
    sub_node[kRollCoefLabel] = yaml::format(rollCoef(row));
    sub_node[kPitchCoefLabel] = yaml::format(pitchCoef(row));
    sub_node[kYawCoefLabel] = yaml::format(yawCoef(row));

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
    liftCoef(row, sub_node[kLiftCoefLabel].as<double>());
    dragCoef(row, sub_node[kDragCoefLabel].as<double>());
    sideCoef(row, sub_node[kSideCoefLabel].as<double>());
    rollCoef(row, sub_node[kRollCoefLabel].as<double>());
    pitchCoef(row, sub_node[kPitchCoefLabel].as<double>());
    yawCoef(row, sub_node[kYawCoefLabel].as<double>());
  }
}

int ControlSurfacesWidget::numUnits() const
{
  return rowCount();
}

QString ControlSurfacesWidget::selected() const
{
  const auto row = currentRow();
  return row >= 0 ? linkName(row) : "";
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

  const auto c_lift_delta = new qt::DoubleSpinBox();
  c_lift_delta->setDecimals(kStabilityCoefDecimals);
  c_lift_delta->setValue(0.0);
  c_lift_delta->setSuffix(" /rad");
  setCellWidget(row, kLiftCoefCol, c_lift_delta);

  const auto c_drag_delta = new qt::DoubleSpinBox();
  c_drag_delta->setDecimals(kStabilityCoefDecimals);
  c_drag_delta->setValue(0.0);
  c_drag_delta->setSuffix(" /rad");
  setCellWidget(row, kDragCoefCol, c_drag_delta);

  const auto c_side_delta = new qt::DoubleSpinBox();
  c_side_delta->setDecimals(kStabilityCoefDecimals);
  c_side_delta->setValue(0.0);
  c_side_delta->setSuffix(" /rad");
  setCellWidget(row, kSideCoefCol, c_side_delta);

  const auto c_roll_delta = new qt::DoubleSpinBox();
  c_roll_delta->setDecimals(kStabilityCoefDecimals);
  c_roll_delta->setValue(0.0);
  c_roll_delta->setSuffix(" /rad");
  setCellWidget(row, kRollCoefCol, c_roll_delta);

  const auto c_pitch_delta = new qt::DoubleSpinBox();
  c_pitch_delta->setDecimals(kStabilityCoefDecimals);
  c_pitch_delta->setValue(0.0);
  c_pitch_delta->setSuffix(" /rad");
  setCellWidget(row, kPitchCoefCol, c_pitch_delta);

  const auto c_yaw_delta = new qt::DoubleSpinBox();
  c_yaw_delta->setDecimals(kStabilityCoefDecimals);
  c_yaw_delta->setValue(0.0);
  c_yaw_delta->setSuffix(" /rad");
  setCellWidget(row, kYawCoefCol, c_yaw_delta);
}

void ControlSurfacesWidget::remove(const QString& link_name)
{
  const auto row = find(link_name);
  removeRow(row);
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

double ControlSurfacesWidget::liftCoef(int row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(row, kLiftCoefCol));
  return cell->value();
}

double ControlSurfacesWidget::dragCoef(int row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(row, kDragCoefCol));
  return cell->value();
}

double ControlSurfacesWidget::sideCoef(int row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(row, kSideCoefCol));
  return cell->value();
}

double ControlSurfacesWidget::rollCoef(int row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(row, kRollCoefCol));
  return cell->value();
}

double ControlSurfacesWidget::pitchCoef(int row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(row, kPitchCoefCol));
  return cell->value();
}

double ControlSurfacesWidget::yawCoef(int row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(row, kYawCoefCol));
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

void ControlSurfacesWidget::liftCoef(int row, double value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(row, kLiftCoefCol));
  return cell->setValue(value);
}

void ControlSurfacesWidget::dragCoef(int row, double value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(row, kDragCoefCol));
  return cell->setValue(value);
}

void ControlSurfacesWidget::sideCoef(int row, double value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(row, kSideCoefCol));
  return cell->setValue(value);
}

void ControlSurfacesWidget::rollCoef(int row, double value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(row, kRollCoefCol));
  return cell->setValue(value);
}

void ControlSurfacesWidget::pitchCoef(int row, double value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(row, kPitchCoefCol));
  return cell->setValue(value);
}

void ControlSurfacesWidget::yawCoef(int row, double value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(row, kYawCoefCol));
  return cell->setValue(value);
}
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
