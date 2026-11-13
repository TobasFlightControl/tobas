// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/control_surfaces.hpp"

#include <QDebug>
#include <QHeaderView>
#include <QLabel>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_yaml_tools/format.hpp>

#include "tobas_setup_assistant/setting_tabs/fixed_wing/constants.hpp"

namespace tobas::gui::sa::fw
{
namespace
{
/* Columns */
constexpr int kLinkNameCol = 0;
constexpr int kJointNameCol = kLinkNameCol + 1;
constexpr int kLiftCoefCol = kJointNameCol + 1;
constexpr int kDragCoefCol = kLiftCoefCol + 1;
constexpr int kSideCoefCol = kDragCoefCol + 1;
constexpr int kRollCoefCol = kSideCoefCol + 1;
constexpr int kPitchCoefCol = kRollCoefCol + 1;
constexpr int kYawCoefCol = kPitchCoefCol + 1;
constexpr int kNumCols = kYawCoefCol + 1;

/* Labels */
constexpr char kLinkNameLabel[] = "Link Name";
constexpr char kJointNameLabel[] = "Joint Name";
constexpr char kLiftCoefLabel[] = "Lift Coef";
constexpr char kDragCoefLabel[] = "Drag Coef";
constexpr char kSideCoefLabel[] = "Side Coef";
constexpr char kRollCoefLabel[] = "Roll Coef";
constexpr char kPitchCoefLabel[] = "Pitch Coef";
constexpr char kYawCoefLabel[] = "Yaw Coef";
}  // namespace

ControlSurfacesWidget::ControlSurfacesWidget(const uadf::Model& _uadf) : super(0, kNumCols), uadf_(_uadf)
{
  setHorizontalHeaderLabels(
    {
      kLinkNameLabel,
      kJointNameLabel,
      kLiftCoefLabel,
      kDragCoefLabel,
      kSideCoefLabel,
      kRollCoefLabel,
      kPitchCoefLabel,
      kYawCoefLabel,
    });
  setColumnsWidth(120);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
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

void ControlSurfacesWidget::load(const YAML::Node& _node)
{
  for (const auto& pair : _node) {
    const auto link_name = pair.first.as<QString>();
    const auto& sub_node = pair.second;

    const auto row = find(link_name);
    if (row < 0) {
      throw std::runtime_error("Failed to find CS link '" + link_name.toStdString() + "'.");
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

int ControlSurfacesWidget::find(const QString& _link_name) const
{
  for (int row = 0; row < rowCount(); ++row) {
    if (linkName(row) == _link_name) {
      return row;
    }
  }

  qWarning() << _link_name << "is not selected as a control surface.";
  return -1;
}

void ControlSurfacesWidget::add(const QString& _link_name)
{
  const auto joint = uadf_.urdf->getLink(_link_name.toStdString())->parent_joint;

  const auto row = rowCount();
  insertRow(row);

  const auto link_name_label = new QLabel(_link_name);
  link_name_label->setFont(qt::DefaultFont(cmn::kBodyPSize));
  link_name_label->setAlignment(Qt::AlignCenter);
  setCellWidget(row, kLinkNameCol, link_name_label);

  const auto joint_name_label = new QLabel(QString::fromStdString(joint->name));
  joint_name_label->setFont(qt::DefaultFont(cmn::kBodyPSize));
  joint_name_label->setAlignment(Qt::AlignCenter);
  setCellWidget(row, kJointNameCol, joint_name_label);

  const auto c_lift_delta = new qt::DoubleSpinBox();
  c_lift_delta->setDecimals(kStabilityCoefDecimals);
  c_lift_delta->setSuffix(" /rad");
  setCellWidget(row, kLiftCoefCol, c_lift_delta);

  const auto c_drag_delta = new qt::DoubleSpinBox();
  c_drag_delta->setDecimals(kStabilityCoefDecimals);
  c_drag_delta->setSuffix(" /rad");
  setCellWidget(row, kDragCoefCol, c_drag_delta);

  const auto c_side_delta = new qt::DoubleSpinBox();
  c_side_delta->setDecimals(kStabilityCoefDecimals);
  c_side_delta->setSuffix(" /rad");
  setCellWidget(row, kSideCoefCol, c_side_delta);

  const auto c_roll_delta = new qt::DoubleSpinBox();
  c_roll_delta->setDecimals(kStabilityCoefDecimals);
  c_roll_delta->setSuffix(" /rad");
  setCellWidget(row, kRollCoefCol, c_roll_delta);

  const auto c_pitch_delta = new qt::DoubleSpinBox();
  c_pitch_delta->setDecimals(kStabilityCoefDecimals);
  c_pitch_delta->setSuffix(" /rad");
  setCellWidget(row, kPitchCoefCol, c_pitch_delta);

  const auto c_yaw_delta = new qt::DoubleSpinBox();
  c_yaw_delta->setDecimals(kStabilityCoefDecimals);
  c_yaw_delta->setSuffix(" /rad");
  setCellWidget(row, kYawCoefCol, c_yaw_delta);

  setToDefault(row);
}

void ControlSurfacesWidget::remove(const QString& _link_name)
{
  const auto row = find(_link_name);
  removeRow(row);
}

QString ControlSurfacesWidget::linkName(int _row) const
{
  const auto cell = qt::qConstPointerCast<QLabel>(cellWidget(_row, kLinkNameCol));
  return cell->text();
}

QString ControlSurfacesWidget::jointName(int _row) const
{
  const auto cell = qt::qConstPointerCast<QLabel>(cellWidget(_row, kJointNameCol));
  return cell->text();
}

double ControlSurfacesWidget::liftCoef(int _row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kLiftCoefCol));
  return cell->value();
}

double ControlSurfacesWidget::dragCoef(int _row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kDragCoefCol));
  return cell->value();
}

double ControlSurfacesWidget::sideCoef(int _row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kSideCoefCol));
  return cell->value();
}

double ControlSurfacesWidget::rollCoef(int _row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kRollCoefCol));
  return cell->value();
}

double ControlSurfacesWidget::pitchCoef(int _row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kPitchCoefCol));
  return cell->value();
}

double ControlSurfacesWidget::yawCoef(int _row) const
{
  const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kYawCoefCol));
  return cell->value();
}

void ControlSurfacesWidget::linkName(int _row, const QString& _text)
{
  const auto cell = qt::qPointerCast<QLabel>(cellWidget(_row, kLinkNameCol));
  return cell->setText(_text);
}

void ControlSurfacesWidget::jointName(int _row, const QString& _text)
{
  const auto cell = qt::qPointerCast<QLabel>(cellWidget(_row, kJointNameCol));
  return cell->setText(_text);
}

void ControlSurfacesWidget::liftCoef(int _row, double _value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kLiftCoefCol));
  return cell->setValue(_value);
}

void ControlSurfacesWidget::dragCoef(int _row, double _value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kDragCoefCol));
  return cell->setValue(_value);
}

void ControlSurfacesWidget::sideCoef(int _row, double _value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kSideCoefCol));
  return cell->setValue(_value);
}

void ControlSurfacesWidget::rollCoef(int _row, double _value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kRollCoefCol));
  return cell->setValue(_value);
}

void ControlSurfacesWidget::pitchCoef(int _row, double _value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kPitchCoefCol));
  return cell->setValue(_value);
}

void ControlSurfacesWidget::yawCoef(int _row, double _value)
{
  const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(cellWidget(_row, kYawCoefCol));
  return cell->setValue(_value);
}

void ControlSurfacesWidget::setToDefault(int _row)
{
  liftCoef(_row, 0.0);
  dragCoef(_row, 0.0);
  sideCoef(_row, 0.0);
  rollCoef(_row, 0.0);
  pitchCoef(_row, 0.0);
  yawCoef(_row, 0.0);
}
}  // namespace tobas::gui::sa::fw
