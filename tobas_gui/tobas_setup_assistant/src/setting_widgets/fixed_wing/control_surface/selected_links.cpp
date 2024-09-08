#include <QLabel>
#include <QDebug>

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "tobas_setup_assistant/setting_tabs/fixed_wing/control_surface/selected_links.hpp"
#include "tobas_setup_assistant/setting_tabs/fixed_wing/constants.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace gui
{
namespace setup_assistant
{
namespace fixed_wing
{
SelectedLinksWidget::SelectedLinksWidget(const RobotInfo& robot) : super(0, kNumCols), robot_(robot)
{
  setHorizontalHeaderLabels({
    kLinkNameLabel,
    kJointNameLabel,
    kMinAngleLabel,
    kMaxAngleLabel,
    kMaxAngleRateLabel,
    kLiftCoefLabel,
    kDragCoefLabel,
    kSideCoefLabel,
    kRollCoefLabel,
    kPitchCoefLabel,
    kYawCoefLabel,
  });

  for (int col = 0; col < kNumCols; ++col)
    setColumnWidth(col, kColWidth);
}

void SelectedLinksWidget::updateInternalDataStructures()
{
  removeAll();
}

YAML::Node SelectedLinksWidget::dump(const QString& link_name) const
{
  YAML::Node node(YAML::NodeType::Map);

  const auto row = find(link_name);

  node[kLinkNameLabel] = linkName(row);
  node[kJointNameLabel] = jointName(row);
  node[kMinAngleLabel] = minAngle(row);
  node[kMaxAngleLabel] = maxAngle(row);
  node[kMaxAngleRateLabel] = maxAngleRate(row);
  node[kLiftCoefLabel] = liftCoef(row);
  node[kDragCoefLabel] = dragCoef(row);
  node[kSideCoefLabel] = sideCoef(row);
  node[kRollCoefLabel] = rollCoef(row);
  node[kPitchCoefLabel] = pitchCoef(row);
  node[kYawCoefLabel] = yawCoef(row);

  return node;
}

void SelectedLinksWidget::load(const QString& link_name, const YAML::Node& node)
{
  const auto row = find(link_name);

  linkName(row, node[kLinkNameLabel].as<QString>());
  jointName(row, node[kJointNameLabel].as<QString>());
  minAngle(row, node[kMinAngleLabel].as<double>());
  maxAngle(row, node[kMaxAngleLabel].as<double>());
  maxAngleRate(row, node[kMaxAngleRateLabel].as<double>());
  liftCoef(row, node[kLiftCoefLabel].as<double>());
  dragCoef(row, node[kDragCoefLabel].as<double>());
  sideCoef(row, node[kSideCoefLabel].as<double>());
  rollCoef(row, node[kRollCoefLabel].as<double>());
  pitchCoef(row, node[kPitchCoefLabel].as<double>());
  yawCoef(row, node[kYawCoefLabel].as<double>());
}

QString SelectedLinksWidget::selected() const
{
  const auto row = currentRow();
  return row >= 0 ? linkName(row) : "";
}

int SelectedLinksWidget::find(const QString& link_name) const
{
  for (int row = 0; row < rowCount(); ++row)
    if (linkName(row) == link_name)
      return row;

  qWarning() << link_name << " is not selected as a control surface.";
  return -1;
}

void SelectedLinksWidget::add(const QString& link_name)
{
  const auto seg_it = robot_.tree().getSegment(link_name.toStdString());
  const auto& joint = seg_it->second.segment.joint();

  const auto row = rowCount();
  insertRow(row);

  const auto link_name_label = new QLabel(link_name);
  link_name_label->setFont(qt::DefaultFont(kBodyPSize));
  link_name_label->setAlignment(Qt::AlignCenter);
  setCellWidget(row, kLinkNameCol, link_name_label);

  const auto joint_name_label = new QLabel(QString::fromStdString(joint.name));
  joint_name_label->setFont(qt::DefaultFont(kBodyPSize));
  joint_name_label->setAlignment(Qt::AlignCenter);
  setCellWidget(row, kJointNameCol, joint_name_label);

  const auto min_angle = new qt::DoubleSpinBox();
  min_angle->setMinimum(-kAngleLimit);
  min_angle->setMaximum(0.);
  min_angle->setDecimals(3);
  min_angle->setSuffix(" rad");
  min_angle->setValue(joint.lower_limit);
  setCellWidget(row, kMinAngleCol, min_angle);

  const auto max_angle = new qt::DoubleSpinBox();
  max_angle->setMinimum(0.);
  max_angle->setMaximum(kAngleLimit);
  max_angle->setDecimals(3);
  max_angle->setSuffix(" rad");
  max_angle->setValue(joint.upper_limit);
  setCellWidget(row, kMaxAngleCol, max_angle);

  const auto max_angle_rate = new qt::DoubleSpinBox();
  max_angle_rate->setDecimals(3);
  max_angle_rate->setMinimum(1e-3);
  max_angle_rate->setValue(joint.max_velocity);
  max_angle_rate->setSuffix(" rad/s");
  setCellWidget(row, kMaxAngleRateCol, max_angle_rate);

  const auto c_lift_delta = new qt::DoubleSpinBox();
  c_lift_delta->setDecimals(kStabilityCoefDecimals);
  c_lift_delta->setValue(0.);
  c_lift_delta->setSuffix(" /rad");
  setCellWidget(row, kLiftCoefCol, c_lift_delta);

  const auto c_drag_delta = new qt::DoubleSpinBox();
  c_drag_delta->setDecimals(kStabilityCoefDecimals);
  c_drag_delta->setValue(0.);
  c_drag_delta->setSuffix(" /rad");
  setCellWidget(row, kDragCoefCol, c_drag_delta);

  const auto c_side_delta = new qt::DoubleSpinBox();
  c_side_delta->setDecimals(kStabilityCoefDecimals);
  c_side_delta->setValue(0.);
  c_side_delta->setSuffix(" /rad");
  setCellWidget(row, kSideCoefCol, c_side_delta);

  const auto c_roll_delta = new qt::DoubleSpinBox();
  c_roll_delta->setDecimals(kStabilityCoefDecimals);
  c_roll_delta->setValue(0.);
  c_roll_delta->setSuffix(" /rad");
  setCellWidget(row, kRollCoefCol, c_roll_delta);

  const auto c_pitch_delta = new qt::DoubleSpinBox();
  c_pitch_delta->setDecimals(kStabilityCoefDecimals);
  c_pitch_delta->setValue(0.);
  c_pitch_delta->setSuffix(" /rad");
  setCellWidget(row, kPitchCoefCol, c_pitch_delta);

  const auto c_yaw_delta = new qt::DoubleSpinBox();
  c_yaw_delta->setDecimals(kStabilityCoefDecimals);
  c_yaw_delta->setValue(0.);
  c_yaw_delta->setSuffix(" /rad");
  setCellWidget(row, kYawCoefCol, c_yaw_delta);
}

void SelectedLinksWidget::remove(const QString& link_name)
{
  const auto row = find(link_name);
  removeRow(row);
}

QString SelectedLinksWidget::linkName(int row) const
{
  const auto cell = qobject_cast<QLabel*>(cellWidget(row, kLinkNameCol));
  return cell->text();
}

QString SelectedLinksWidget::jointName(int row) const
{
  const auto cell = qobject_cast<QLabel*>(cellWidget(row, kJointNameCol));
  return cell->text();
}

double SelectedLinksWidget::minAngle(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kMinAngleCol));
  return cell->value();
}

double SelectedLinksWidget::maxAngle(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kMaxAngleCol));
  return cell->value();
}

double SelectedLinksWidget::maxAngleRate(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kMaxAngleRateCol));
  return cell->value();
}

double SelectedLinksWidget::liftCoef(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kLiftCoefCol));
  return cell->value();
}

double SelectedLinksWidget::dragCoef(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kDragCoefCol));
  return cell->value();
}

double SelectedLinksWidget::sideCoef(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kSideCoefCol));
  return cell->value();
}

double SelectedLinksWidget::rollCoef(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kRollCoefCol));
  return cell->value();
}

double SelectedLinksWidget::pitchCoef(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kPitchCoefCol));
  return cell->value();
}

double SelectedLinksWidget::yawCoef(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kYawCoefCol));
  return cell->value();
}

void SelectedLinksWidget::linkName(int row, const QString& text)
{
  const auto cell = qobject_cast<QLabel*>(cellWidget(row, kLinkNameCol));
  return cell->setText(text);
}

void SelectedLinksWidget::jointName(int row, const QString& text)
{
  const auto cell = qobject_cast<QLabel*>(cellWidget(row, kJointNameCol));
  return cell->setText(text);
}

void SelectedLinksWidget::minAngle(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kMinAngleCol));
  return cell->setValue(value);
}

void SelectedLinksWidget::maxAngle(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kMaxAngleCol));
  return cell->setValue(value);
}

void SelectedLinksWidget::maxAngleRate(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kMaxAngleRateCol));
  return cell->setValue(value);
}

void SelectedLinksWidget::liftCoef(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kLiftCoefCol));
  return cell->setValue(value);
}

void SelectedLinksWidget::dragCoef(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kDragCoefCol));
  return cell->setValue(value);
}

void SelectedLinksWidget::sideCoef(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kSideCoefCol));
  return cell->setValue(value);
}

void SelectedLinksWidget::rollCoef(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kRollCoefCol));
  return cell->setValue(value);
}

void SelectedLinksWidget::pitchCoef(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kPitchCoefCol));
  return cell->setValue(value);
}

void SelectedLinksWidget::yawCoef(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kYawCoefCol));
  return cell->setValue(value);
}

QStringList SelectedLinksWidget::linkNames() const
{
  QStringList res;
  for (int row = 0; row < rowCount(); ++row)
    res.append(linkName(row));
  return res;
}

QStringList SelectedLinksWidget::jointNames() const
{
  QStringList res;
  for (int row = 0; row < rowCount(); ++row)
    res.append(jointName(row));
  return res;
}
}  // namespace fixed_wing
}  // namespace setup_assistant
}  // namespace gui
