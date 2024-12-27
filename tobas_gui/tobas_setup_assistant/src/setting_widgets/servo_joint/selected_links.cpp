#include <QLabel>
#include <QDebug>
#include <QHeaderView>

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
  horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);  // 内容に合わせて横幅を自動調整

  setHorizontalHeaderLabels({
    kLinkNameLabel,
    kJointNameLabel,
    kHomePosLabel,
    kMinPosLabel,
    kMaxPosLabel,
    kInterfaceLabel,
  });
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
  node[kInterfaceLabel] = commandInterface(row);

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
  commandInterface(row, node[kInterfaceLabel].as<tobas::jnt_cmd_iface_t>());
}

int SelectedLinksWidget::count() const
{
  return rowCount();
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
    case kdl::Joint::ROTATION:
      home_pos->setSuffix(" rad");
      min_pos->setSuffix(" rad");
      max_pos->setSuffix(" rad");
      break;
    case kdl::Joint::TRANSLATION:
      home_pos->setSuffix(" m");
      min_pos->setSuffix(" m");
      max_pos->setSuffix(" m");
      break;
    default:
      throw;
  }

  const auto cmd_iface = new qt::ComboBox();
  cmd_iface->addItems({ kPositionLabel, kVelocityLabel, kEffortLabel });

  const auto row = rowCount();
  insertRow(row);
  setCellWidget(row, kLinkNameCol, link_name_label);
  setCellWidget(row, kJointNameCol, jnt_name_label);
  setCellWidget(row, kHomePosCol, home_pos);
  setCellWidget(row, kMinPosCol, min_pos);
  setCellWidget(row, kMaxPosCol, max_pos);
  setCellWidget(row, kCmdIfaceCol, cmd_iface);
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
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kMinPosCol));
  return cell->value();
}

double SelectedLinksWidget::maxPosition(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(cellWidget(row, kMaxPosCol));
  return cell->value();
}

tobas::jnt_cmd_iface_t SelectedLinksWidget::commandInterface(int row) const
{
  const auto cell = qobject_cast<qt::ComboBox*>(cellWidget(row, kCmdIfaceCol));
  const auto text = cell->currentText();

  if (text == kPositionLabel)
    return tobas::jnt_cmd_iface_t::POSITION;
  else if (text == kVelocityLabel)
    return tobas::jnt_cmd_iface_t::VELOCITY;
  else if (text == kEffortLabel)
    return tobas::jnt_cmd_iface_t::EFFORT;
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

void SelectedLinksWidget::commandInterface(int row, tobas::jnt_cmd_iface_t value)
{
  QString text;
  switch (value)
  {
    case tobas::jnt_cmd_iface_t::POSITION:
      text = kPositionLabel;
      break;
    case tobas::jnt_cmd_iface_t::VELOCITY:
      text = kVelocityLabel;
      break;
    case tobas::jnt_cmd_iface_t::EFFORT:
      text = kEffortLabel;
      break;
    default:
      throw;
  }

  const auto cell = qobject_cast<qt::ComboBox*>(cellWidget(row, kCmdIfaceCol));
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
