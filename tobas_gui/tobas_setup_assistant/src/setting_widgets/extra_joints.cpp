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

namespace gui
{
namespace sa
{
ExtraJointsWidget::ExtraJointsWidget(const RobotInfo& robot) : robot_(robot)
{
  table_ = new qt::TableWidget(0, kNumCols);
  table_->setHeaderSectionsClickable(false);
  table_->setHorizontalHeaderLabels({ kLinkNameLabel, kJointNameLabel, kRoleLabel, kCmdIfaceLabel, kHomePosLabel });
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);  // 内容に合わせて横幅を自動調整
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
  return "";  // TODO
}

void ExtraJointsWidget::updateInternalDataStructures()
{
  clear();

  for (const auto& [link_name, elem] : robot_.tree().getSegments()) {
    if (link_name == robot_.tree().getRootName()) {
      continue;
    }

    const auto& joint = elem.segment.joint();

    // 可動関節でなければスキップ
    if (joint.type == kdl::Joint::FIXED) {
      continue;
    }

    // TODO: 直動ジョイントにも対応
    if (joint.type == kdl::Joint::TRANSLATION) {
      qt::qWarnBox(this, "Translational joint is not supported yet.");
      continue;
    }

    // UADFに記載のあるジョイントはスキップ
    const auto& uadf = robot_.uadf();
    if (uadf.thrusts.contains(joint.name)) {
      continue;
    }
    if (uadf.control_surfaces.contains(joint.name)) {
      continue;
    }
    if (uadf.tilts.contains(joint.name)) {
      continue;
    }

    // テーブルに追加
    addLink(link_name);
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

    sub_node[kRoleLabel] = roleWidget(row)->currentText();
    sub_node[kCmdIfaceLabel] = commandIfaceWidget(row)->currentText();
    sub_node[kHomePosLabel] = homePositionWidget(row)->value();

    node[getLinkName(row)] = sub_node;
  }

  return node;
}

void ExtraJointsWidget::load(const YAML::Node& node)
{
  // 各フィールドの値と有効無効を設定
  for (const auto& pair : node) {
    const auto link_name = pair.first.as<QString>();
    const auto& sub_node = pair.second;

    const auto row = findLink(link_name);
    TOBAS_CHECK(row >= 0);

    roleWidget(row)->setCurrentText(sub_node[kRoleLabel].as<QString>());
    commandIfaceWidget(row)->setCurrentText(sub_node[kCmdIfaceLabel].as<QString>());
    homePositionWidget(row)->setValue(sub_node[kHomePosLabel].as<int>());

    updateEnability(row);
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

tobas::jnt_role_t ExtraJointsWidget::getRole(int row) const
{
  const auto text = roleWidget(row)->currentText();

  if (text == kRoleLabel_LandingGear) {
    return tobas::jnt_role_t::LANDING_GEAR;
  }
  else if (text == kRoleLabel_PassiveWheel) {
    return tobas::jnt_role_t::PASSIVE_WHEEL;
  }
  else if (text == kRoleLabel_Manipulation) {
    return tobas::jnt_role_t::MANIPULATION;
  }
  else if (text == kRoleLabel_Other) {
    return tobas::jnt_role_t::OTHER;
  }
  else {
    throw;
  }
}

tobas::jnt_cmd_iface_t ExtraJointsWidget::getCommandInterface(int row) const
{
  const auto text = commandIfaceWidget(row)->currentText();

  if (text == kCmdIfaceLabel_Position) {
    return tobas::jnt_cmd_iface_t::POSITION;
  }
  else if (text == kCmdIfaceLabel_Velocity) {
    return tobas::jnt_cmd_iface_t::VELOCITY;
  }
  else if (text == kCmdIfaceLabel_Effort) {
    return tobas::jnt_cmd_iface_t::EFFORT;
  }
  else if (text == kCmdIfaceLabel_None) {
    return tobas::jnt_cmd_iface_t::NONE;
  }
  else {
    throw;
  }
}

double ExtraJointsWidget::getHomePosition(int row) const
{
  return tobas_std::deg2rad(homePositionWidget(row)->value());
}

void ExtraJointsWidget::setRole(int row, tobas::jnt_role_t value)
{
  QString text;
  switch (value) {
    case tobas::jnt_role_t::LANDING_GEAR:
      text = kRoleLabel_LandingGear;
      break;
    case tobas::jnt_role_t::PASSIVE_WHEEL:
      text = kRoleLabel_PassiveWheel;
      break;
    case tobas::jnt_role_t::MANIPULATION:
      text = kRoleLabel_Manipulation;
      break;
    case tobas::jnt_role_t::OTHER:
      text = kRoleLabel_Other;
      break;
    default:
      throw;
  }

  roleWidget(row)->setCurrentText(text);
}

void ExtraJointsWidget::setCommandInterface(int row, tobas::jnt_cmd_iface_t value)
{
  QString text;
  switch (value) {
    case tobas::jnt_cmd_iface_t::POSITION:
      text = kCmdIfaceLabel_Position;
      break;
    case tobas::jnt_cmd_iface_t::VELOCITY:
      text = kCmdIfaceLabel_Velocity;
      break;
    case tobas::jnt_cmd_iface_t::EFFORT:
      text = kCmdIfaceLabel_Effort;
      break;
    case tobas::jnt_cmd_iface_t::NONE:
      text = kCmdIfaceLabel_None;
      break;
    default:
      throw;
  }

  commandIfaceWidget(row)->setCurrentText(text);
}

void ExtraJointsWidget::setHomePosition(int row, double value)
{
  homePositionWidget(row)->setValue(std::round(tobas_std::rad2deg(value)));
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

  qWarning() << "Link " << link_name << " is not found.";
  return -1;
}

int ExtraJointsWidget::findJoint(const QString& joint_name) const
{
  for (int row = 0; row < table_->rowCount(); ++row) {
    if (getJointName(row) == joint_name) {
      return row;
    }
  }

  qWarning() << "Joint " << joint_name << " is not found.";
  return -1;
}

QLabel* ExtraJointsWidget::linkNameWidget(int row)
{
  return qt::qPointerCast<QLabel>(table_->cellWidget(row, kLinkNameCol));
}

QLabel* ExtraJointsWidget::jointNameWidget(int row)
{
  return qt::qPointerCast<QLabel>(table_->cellWidget(row, kJointNameCol));
}

qt::ComboBox* ExtraJointsWidget::roleWidget(int row)
{
  return qt::qPointerCast<qt::ComboBox>(table_->cellWidget(row, kRoleCol));
}

qt::ComboBox* ExtraJointsWidget::commandIfaceWidget(int row)
{
  return qt::qPointerCast<qt::ComboBox>(table_->cellWidget(row, kCmdIfaceCol));
}

qt::SpinBox* ExtraJointsWidget::homePositionWidget(int row)
{
  return qt::qPointerCast<qt::SpinBox>(table_->cellWidget(row, kHomePosCol));
}

const QLabel* ExtraJointsWidget::linkNameWidget(int row) const
{
  return qt::qConstPointerCast<QLabel>(table_->cellWidget(row, kLinkNameCol));
}

const QLabel* ExtraJointsWidget::jointNameWidget(int row) const
{
  return qt::qConstPointerCast<QLabel>(table_->cellWidget(row, kJointNameCol));
}

const qt::ComboBox* ExtraJointsWidget::roleWidget(int row) const
{
  return qt::qConstPointerCast<qt::ComboBox>(table_->cellWidget(row, kRoleCol));
}

const qt::ComboBox* ExtraJointsWidget::commandIfaceWidget(int row) const
{
  return qt::qConstPointerCast<qt::ComboBox>(table_->cellWidget(row, kCmdIfaceCol));
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
  roleWidget(row)->setCurrentText(kRoleLabel_Other);

  setDefaultValues(row);
  updateEnability(row);
}

void ExtraJointsWidget::setDefaultValues(int row)
{
  // 役割に応じてコマンドインターフェースとハードウェアインターフェースを設定
  switch (getRole(row)) {
    case tobas::jnt_role_t::LANDING_GEAR:
      commandIfaceWidget(row)->setCurrentText(kCmdIfaceLabel_Position);
      break;
    case tobas::jnt_role_t::PASSIVE_WHEEL:
      commandIfaceWidget(row)->setCurrentText(kCmdIfaceLabel_None);
      break;
    case tobas::jnt_role_t::MANIPULATION:
      commandIfaceWidget(row)->setCurrentText(kCmdIfaceLabel_Position);
      break;
    case tobas::jnt_role_t::OTHER:
      commandIfaceWidget(row)->setCurrentText(kCmdIfaceLabel_None);
      break;
    default:
      throw;
  }

  // 共通のデフォルト値
  homePositionWidget(row)->setValue(0);
}

void ExtraJointsWidget::updateEnability(int row)
{
  // 役割によるフィールド
  switch (getRole(row)) {
    case tobas::jnt_role_t::LANDING_GEAR:
      roleWidget(row)->setEnabled(true);
      commandIfaceWidget(row)->setEnabled(true);
      homePositionWidget(row)->setEnabled(true);
      break;
    case tobas::jnt_role_t::PASSIVE_WHEEL:
      roleWidget(row)->setEnabled(true);
      commandIfaceWidget(row)->setEnabled(false);
      homePositionWidget(row)->setEnabled(false);
      break;
    case tobas::jnt_role_t::MANIPULATION:
      roleWidget(row)->setEnabled(true);
      commandIfaceWidget(row)->setEnabled(true);
      homePositionWidget(row)->setEnabled(true);
      break;
    case tobas::jnt_role_t::OTHER:
      roleWidget(row)->setEnabled(true);
      commandIfaceWidget(row)->setEnabled(false);
      homePositionWidget(row)->setEnabled(false);
      break;
    default:
      throw;
  }
}

void ExtraJointsWidget::addLink(const std::string& link_name)
{
  const auto row = table_->rowCount();

  const auto seg_it = robot_.tree().getSegment(link_name);
  const auto& joint = seg_it->second.segment.joint();

  // Name
  const auto link_name_label = new QLabel(QString::fromStdString(link_name));
  const auto joint_name_label = new QLabel(QString::fromStdString(joint.name));

  // Role
  const auto role = new qt::ComboBox();
  role->addItems({
    kRoleLabel_LandingGear,
    kRoleLabel_PassiveWheel,
    kRoleLabel_Manipulation,
    kRoleLabel_Other,
  });

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
  home_pos->setMinimum(std::round(tobas_std::rad2deg(std::isinf(joint.lower_limit) ? -M_PI : joint.lower_limit)));
  home_pos->setMaximum(std::round(tobas_std::rad2deg(std::isinf(joint.upper_limit) ? M_PI : joint.upper_limit)));
  home_pos->setSuffix(" deg");

  // Insert table row
  table_->insertRow(row);
  table_->setCellWidget(row, kLinkNameCol, link_name_label);
  table_->setCellWidget(row, kJointNameCol, joint_name_label);
  table_->setCellWidget(row, kRoleCol, role);
  table_->setCellWidget(row, kCmdIfaceCol, cmd_iface);
  table_->setCellWidget(row, kHomePosCol, home_pos);

  // Reset
  reset(row);

  // Connection
  connect(role, &qt::ComboBox::currentTextChanged, std::bind(&self::onRoleChanged, this, row));
}

void ExtraJointsWidget::onRoleChanged(int row)
{
  setDefaultValues(row);
  updateEnability(row);
}
}  // namespace sa
}  // namespace gui
