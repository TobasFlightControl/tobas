// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/extra_joints.hpp"

#include <QDebug>
#include <QHeaderView>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/constants.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace
{
// Columns
constexpr int kLinkNameCol = 0;
constexpr int kJointNameCol = kLinkNameCol + 1;
constexpr int kCmdIfaceCol = kJointNameCol + 1;
constexpr int kHomePosCol = kCmdIfaceCol + 1;
constexpr int kNumCols = kHomePosCol + 1;

// Field Labels
constexpr char kLinkNameLabel[] = "Link Name";
constexpr char kJointNameLabel[] = "Joint Name";
constexpr char kCmdIfaceLabel[] = "Command Interface";
constexpr char kHomePosLabel[] = "Home Position";

// Command Interface Labels
constexpr char kCmdIfaceLabel_Position[] = "Position";
constexpr char kCmdIfaceLabel_Velocity[] = "Velocity";
constexpr char kCmdIfaceLabel_Effort[] = "Effort";
constexpr char kCmdIfaceLabel_None[] = "None";
}  // namespace

ExtraJointsWidget::ExtraJointsWidget(const uadf::Model& uadf, const kdl::Tree& tree) : uadf_(uadf), tree_(tree)
{
  table_ = new qt::TableWidget(0, kNumCols);
  table_->setHeaderSectionsClickable(false);
  table_->setHorizontalHeaderLabels({ kLinkNameLabel, kJointNameLabel, kCmdIfaceLabel, kHomePosLabel });
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  addWidget(table_);
}

const char* ExtraJointsWidget::name() const
{
  return "Extra Joints";
}

const char* ExtraJointsWidget::title() const
{
  return "Configure Extra Joints";
}

const char* ExtraJointsWidget::description() const
{
  return "Configure any joints that are not directly involved in flight control. "
         "Complete every field in the table below with the appropriate values.";
}

void ExtraJointsWidget::updateInternalDataStructures()
{
  clear();

  for (const auto& [link_name, elem] : tree_.getSegments()) {
    if (link_name == tree_.getRootName()) {
      continue;
    }

    const auto& joint = elem.segment.joint();

    // Skip non-movable joints.
    if (joint.type == kdl::Joint::kFixed) {
      continue;
    }

    // TODO: Support prismatic joints.
    if (joint.type == kdl::Joint::kTranslation) {
      qt::qWarnBox(this, "Translational joint is not supported yet.");
      continue;
    }

    // Skip joints listed in UADF.
    if (uadf_.thrusts.contains(joint.name)) {
      continue;
    }
    if (uadf_.control_surfaces.contains(joint.name)) {
      continue;
    }
    if (uadf_.tilts.contains(joint.name)) {
      continue;
    }

    // Add to the table.
    addLink(link_name);
  }
}

void ExtraJointsWidget::setToDefaults()
{
  for (int row = 0; row < table_->rowCount(); ++row) {
    reset(row);
  }
}

bool ExtraJointsWidget::isValid()
{
  return true;
}

YAML::Node ExtraJointsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int row = 0; row < table_->rowCount(); ++row) {
    YAML::Node sub_node(YAML::NodeType::Map);

    sub_node[kCmdIfaceLabel] = commandIfaceWidget(row)->currentText();
    sub_node[kHomePosLabel] = homePositionWidget(row)->value();

    node[getLinkName(row)] = sub_node;
  }

  return node;
}

void ExtraJointsWidget::load(const YAML::Node& node)
{
  // Set each field value and enabled state.
  for (const auto& pair : node) {
    const auto link_name = pair.first.as<QString>();
    const auto& sub_node = pair.second;

    const auto row = findLink(link_name);
    TOBAS_CHECK(row >= 0);

    commandIfaceWidget(row)->setCurrentText(sub_node[kCmdIfaceLabel].as<QString>());
    homePositionWidget(row)->setValue(sub_node[kHomePosLabel].as<int>());
  }
}

QString ExtraJointsWidget::getLinkName(int row) const
{
  return linkNameWidget(row)->text();
}

QString ExtraJointsWidget::getJointName(int row) const
{
  return jointNameWidget(row)->text();
}

JointCommandInterface ExtraJointsWidget::getCommandInterface(int row) const
{
  const auto text = commandIfaceWidget(row)->currentText();

  if (text == kCmdIfaceLabel_Position) {
    return JointCommandInterface::kPosition;
  }
  else if (text == kCmdIfaceLabel_Velocity) {
    return JointCommandInterface::kVelocity;
  }
  else if (text == kCmdIfaceLabel_Effort) {
    return JointCommandInterface::kEffort;
  }
  else if (text == kCmdIfaceLabel_None) {
    return JointCommandInterface::kNone;
  }
  else {
    throw;
  }
}

JointRole ExtraJointsWidget::getRole(int row) const
{
  const auto cmd_iface = getCommandInterface(row);
  return cmd_iface == JointCommandInterface::kNone ? JointRole::kUserPassive : JointRole::kUserActive;
}

double ExtraJointsWidget::getHomePosition(int row) const
{
  return st::deg2rad(homePositionWidget(row)->value());
}

void ExtraJointsWidget::setCommandInterface(int row, JointCommandInterface value)
{
  QString text;
  switch (value) {
    case JointCommandInterface::kPosition:
      text = kCmdIfaceLabel_Position;
      break;
    case JointCommandInterface::kVelocity:
      text = kCmdIfaceLabel_Velocity;
      break;
    case JointCommandInterface::kEffort:
      text = kCmdIfaceLabel_Effort;
      break;
    case JointCommandInterface::kNone:
      text = kCmdIfaceLabel_None;
      break;
    default:
      throw;
  }

  commandIfaceWidget(row)->setCurrentText(text);
}

void ExtraJointsWidget::setHomePosition(int row, double value)
{
  homePositionWidget(row)->setValue(std::round(st::rad2deg(value)));
}

int ExtraJointsWidget::numJoints() const
{
  return table_->rowCount();
}

int ExtraJointsWidget::findLink(const QString& link_name) const
{
  for (int row = 0; row < table_->rowCount(); ++row) {
    if (getLinkName(row) == link_name) {
      return row;
    }
  }

  qWarning() << "Link" << link_name << "not found.";
  return -1;
}

int ExtraJointsWidget::findJoint(const QString& joint_name) const
{
  for (int row = 0; row < table_->rowCount(); ++row) {
    if (getJointName(row) == joint_name) {
      return row;
    }
  }

  qWarning() << "Joint" << joint_name << "not found.";
  return -1;
}

QLabel* ExtraJointsWidget::linkNameWidget(int row)
{
  return qt::qPointerCast<QLabel>(table_->cellWidget(row, kLinkNameCol));
}

const QLabel* ExtraJointsWidget::linkNameWidget(int row) const
{
  return qt::qConstPointerCast<QLabel>(table_->cellWidget(row, kLinkNameCol));
}

QLabel* ExtraJointsWidget::jointNameWidget(int row)
{
  return qt::qPointerCast<QLabel>(table_->cellWidget(row, kJointNameCol));
}

const QLabel* ExtraJointsWidget::jointNameWidget(int row) const
{
  return qt::qConstPointerCast<QLabel>(table_->cellWidget(row, kJointNameCol));
}

qt::ComboBox* ExtraJointsWidget::commandIfaceWidget(int row)
{
  return qt::qPointerCast<qt::ComboBox>(table_->cellWidget(row, kCmdIfaceCol));
}

const qt::ComboBox* ExtraJointsWidget::commandIfaceWidget(int row) const
{
  return qt::qConstPointerCast<qt::ComboBox>(table_->cellWidget(row, kCmdIfaceCol));
}

qt::SpinBox* ExtraJointsWidget::homePositionWidget(int row)
{
  return qt::qPointerCast<qt::SpinBox>(table_->cellWidget(row, kHomePosCol));
}

const qt::SpinBox* ExtraJointsWidget::homePositionWidget(int row) const
{
  return qt::qConstPointerCast<qt::SpinBox>(table_->cellWidget(row, kHomePosCol));
}

void ExtraJointsWidget::clear()
{
  table_->removeAll();
}

void ExtraJointsWidget::reset(int row)
{
  const auto cmd_iface = commandIfaceWidget(row);
  if (cmd_iface->isEnabled()) {
    cmd_iface->setCurrentText(kCmdIfaceLabel_Position);
  }

  const auto home_pos = homePositionWidget(row);
  if (home_pos->isEnabled()) {
    home_pos->setValue(0);
  }
}

void ExtraJointsWidget::addLink(const std::string& link_name)
{
  const auto row = table_->rowCount();

  const auto seg_it = tree_.getSegment(link_name);
  const auto& joint = seg_it->second.segment.joint();
  const auto& joint_name = joint.name;
  TOBAS_CHECK(joint.type == kdl::Joint::kRotation);

  // Name
  const auto link_name_label = new QLabel(QString::fromStdString(link_name));
  const auto joint_name_label = new QLabel(QString::fromStdString(joint_name));

  // Command Interface
  const auto cmd_iface = new qt::ComboBox();
  cmd_iface->addItems({
    kCmdIfaceLabel_Position,
    kCmdIfaceLabel_Velocity,
    kCmdIfaceLabel_Effort,
    kCmdIfaceLabel_None,
  });

  // Home Position
  const auto home_pos = new qt::SpinBox();
  home_pos->setMinimum(std::isinf(joint.lower_limit) ? -180 : std::round(st::rad2deg(joint.lower_limit)));
  home_pos->setMaximum(std::isinf(joint.upper_limit) ? +180 : std::round(st::rad2deg(joint.upper_limit)));
  home_pos->setSuffix(" deg");

  // Treat the joint as passive if its limits are invalid.
  const auto limits = uadf_.urdf->getJoint(joint_name)->limits;
  if (!limits || limits->lower >= limits->upper || limits->velocity <= 0.0 || limits->effort <= 0.0) {
    cmd_iface->setCurrentText(kCmdIfaceLabel_None);
    cmd_iface->setEnabled(false);
    home_pos->setValue(0);
    home_pos->setEnabled(false);
  }

  // Insert table row.
  table_->insertRow(row);
  table_->setCellWidget(row, kLinkNameCol, link_name_label);
  table_->setCellWidget(row, kJointNameCol, joint_name_label);
  table_->setCellWidget(row, kCmdIfaceCol, cmd_iface);
  table_->setCellWidget(row, kHomePosCol, home_pos);

  // Set to the default values.
  reset(row);
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
