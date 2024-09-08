#include <QLabel>
#include <QDebug>

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "tobas_setup_assistant/setting_tabs/servo_joint/selected_links.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace gui
{
namespace setup_assistant
{
namespace servo_joint
{
SelectedLinksWidget::SelectedLinksWidget(const RobotInfo& robot) : super(0, kNumCols), robot_(robot)
{
  setHorizontalHeaderLabels({
    kLinkNameLabel,
    kJointNameLabel,
    kHomePosLabel,
    kMinPosLabel,
    kMaxPosLabel,
    kCmdTypeLabel,
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
  node[kHomePosLabel] = homePosition(row);
  node[kMinPosLabel] = minPosition(row);
  node[kMaxPosLabel] = maxPosition(row);
  node[kCmdTypeLabel] = commandType(row);

  return node;
}

void SelectedLinksWidget::load(const QString& link_name, const YAML::Node& node)
{
  const auto row = find(link_name);

  linkName(row, node[kLinkNameLabel].as<QString>());
  jointName(row, node[kJointNameLabel].as<QString>());
  homePosition(row, node[kHomePosLabel].as<double>());
  minPosition(row, node[kMinPosLabel].as<double>());
  maxPosition(row, node[kMaxPosLabel].as<double>());
  commandType(row, node[kCmdTypeLabel].as<tobas::joint_control_type_t>());
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

  qWarning() << link_name << " is not selected as a servo joint.";
  return -1;
}

void SelectedLinksWidget::add(const QString& link_name)
{
  const auto seg_it = robot_.tree().getSegment(link_name.toStdString());
  const auto& joint = seg_it->second.segment.joint();

  const auto row = rowCount();
  insertRow(row);

  const auto link_name_label = new QLabel(link_name);
  const auto jnt_name_label = new QLabel(QString::fromStdString(joint.name));

  const auto home_pos = new qt::DoubleSpinBox();
  const auto min_pos = new qt::DoubleSpinBox();
  const auto max_pos = new qt::DoubleSpinBox();

  home_pos->setDecimals(kPosDecimals);
  min_pos->setDecimals(kPosDecimals);
  max_pos->setDecimals(kPosDecimals);

  home_pos->setMinimum(joint.lower_limit);
  min_pos->setMinimum(joint.lower_limit);
  max_pos->setMinimum(joint.lower_limit);

  home_pos->setMaximum(joint.upper_limit);
  min_pos->setMaximum(joint.upper_limit);
  max_pos->setMaximum(joint.upper_limit);

  home_pos->setValue(0.);
  min_pos->setValue(joint.lower_limit);
  max_pos->setValue(joint.upper_limit);

  switch (joint.type)
  {
    case kdl::Joint::RotAxis:
      home_pos->setSuffix(" rad");
      min_pos->setSuffix(" rad");
      max_pos->setSuffix(" rad");
      break;
    case kdl::Joint::TransAxis:
      home_pos->setSuffix(" m");
      min_pos->setSuffix(" m");
      max_pos->setSuffix(" m");
      break;
    default:
      throw;
  }

  const auto cmd_type = new qt::ComboBox();
  cmd_type->addItems({ kPositionLabel, kVelocityLabel, kEffortLabel });

  setCellWidget(row, kLinkNameCol, link_name_label);
  setCellWidget(row, kJointNameCol, jnt_name_label);
  setCellWidget(row, kHomePosCol, home_pos);
  setCellWidget(row, kMinPosCol, min_pos);
  setCellWidget(row, kMaxPosCol, max_pos);
  setCellWidget(row, kCmdTypeCol, cmd_type);
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

double SelectedLinksWidget::homePosition(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kHomePosCol));
  return cell->value();
}

double SelectedLinksWidget::minPosition(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kHomePosCol));
  return cell->value();
}

double SelectedLinksWidget::maxPosition(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kHomePosCol));
  return cell->value();
}

tobas::joint_control_type_t SelectedLinksWidget::commandType(int row) const
{
  const auto cell = qobject_cast<qt::ComboBox*>(cellWidget(row, kHomePosCol));
  const auto text = cell->currentText();

  if (text == kPositionLabel)
    return tobas::joint_control_type_t::POSITION_CONTROL;
  else if (text == kVelocityLabel)
    return tobas::joint_control_type_t::VELOCITY_CONTROL;
  else if (text == kEffortLabel)
    return tobas::joint_control_type_t::EFFORT_CONTROL;
  else
    throw;
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

void SelectedLinksWidget::homePosition(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kHomePosCol));
  cell->setValue(value);
}

void SelectedLinksWidget::minPosition(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kMinPosCol));
  cell->setValue(value);
}

void SelectedLinksWidget::maxPosition(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kMaxPosCol));
  cell->setValue(value);
}

void SelectedLinksWidget::commandType(int row, tobas::joint_control_type_t value)
{
  QString text;
  switch (value)
  {
    case tobas::joint_control_type_t::POSITION_CONTROL:
      text = kPositionLabel;
      break;
    case tobas::joint_control_type_t::VELOCITY_CONTROL:
      text = kVelocityLabel;
      break;
    case tobas::joint_control_type_t::EFFORT_CONTROL:
      text = kEffortLabel;
      break;
    default:
      throw;
  }

  const auto cell = qobject_cast<qt::ComboBox*>(cellWidget(row, kCmdTypeCol));
  cell->setCurrentText(text);
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
}  // namespace servo_joint
}  // namespace setup_assistant
}  // namespace gui
