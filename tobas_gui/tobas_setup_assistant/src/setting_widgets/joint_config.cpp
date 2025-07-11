#include "tobas_setup_assistant/setting_tabs/joint_config.hpp"

#include <QDebug>
#include <QHeaderView>
#include <magic_enum/magic_enum.hpp>

#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/constants.hpp"

namespace gui
{
namespace sa
{
JointConfigurationWidget::JointConfigurationWidget(const RobotInfo& robot) : robot_(robot)
{
  table_ = new qt::TableWidget(0, kNumCols);
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);  // 内容に合わせて横幅を自動調整
  table_->setHorizontalHeaderLabels({ kLinkNameLabel, kJointNameLabel, kRoleLabel, kCmdIfaceLabel, kHomePosLabel });
  addWidget(table_);
}

const char* JointConfigurationWidget::name() const
{
  return "Joint Config";
}

const char* JointConfigurationWidget::title() const
{
  return "Configure Joints";
}

const char* JointConfigurationWidget::description() const
{
  return "";  // TODO
}

void JointConfigurationWidget::updateInternalDataStructures()
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

bool JointConfigurationWidget::isValid()
{
  return true;
}

YAML::Node JointConfigurationWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int row = 0; row < table_->rowCount(); ++row) {
    YAML::Node sub_node(YAML::NodeType::Map);

    sub_node[kRoleLabel] = role_.at(row)->currentText();
    sub_node[kCmdIfaceLabel] = cmd_iface_.at(row)->currentText();
    sub_node[kHomePosLabel] = home_pos_.at(row)->value();

    node[getLinkName(row)] = sub_node;
  }

  return node;
}

void JointConfigurationWidget::load(const YAML::Node& node)
{
  // 各フィールドの値と有効無効を設定
  for (const auto& pair : node) {
    const auto link_name = pair.first.as<QString>();
    const auto& sub_node = pair.second;

    const auto row = findLink(link_name);
    TOBAS_CHECK(row >= 0);

    role_.at(row)->setCurrentText(sub_node[kRoleLabel].as<QString>());
    cmd_iface_.at(row)->setCurrentText(sub_node[kCmdIfaceLabel].as<QString>());
    home_pos_.at(row)->setValue(sub_node[kHomePosLabel].as<int>());

    updateEnability(row);
  }
}

QString JointConfigurationWidget::getLinkName(int row) const
{
  return link_name_.at(row)->text();
}

QString JointConfigurationWidget::getJointName(int row) const
{
  return joint_name_.at(row)->text();
}

tobas::jnt_role_t JointConfigurationWidget::getRole(int row) const
{
  const auto text = role_.at(row)->currentText();

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

tobas::jnt_cmd_iface_t JointConfigurationWidget::getCommandInterface(int row) const
{
  const auto text = cmd_iface_.at(row)->currentText();

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

double JointConfigurationWidget::getHomePosition(int row) const
{
  return tobas_std::deg2rad(home_pos_.at(row)->value());
}

void JointConfigurationWidget::setRole(int row, tobas::jnt_role_t value)
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

  role_.at(row)->setCurrentText(text);
}

void JointConfigurationWidget::setCommandInterface(int row, tobas::jnt_cmd_iface_t value)
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

  cmd_iface_.at(row)->setCurrentText(text);
}

void JointConfigurationWidget::setHomePosition(int row, double value)
{
  home_pos_.at(row)->setValue(std::round(tobas_std::rad2deg(value)));
}

int JointConfigurationWidget::numJoints() const
{
  return table_->rowCount();
}

int JointConfigurationWidget::findLink(const QString& link_name) const
{
  for (int row = 0; row < table_->rowCount(); ++row) {
    if (getLinkName(row) == link_name) {
      return row;
    }
  }

  qWarning() << "Link " << link_name << " is not found.";
  return -1;
}

int JointConfigurationWidget::findJoint(const QString& joint_name) const
{
  for (int row = 0; row < table_->rowCount(); ++row) {
    if (getJointName(row) == joint_name) {
      return row;
    }
  }

  qWarning() << "Joint " << joint_name << " is not found.";
  return -1;
}

void JointConfigurationWidget::clear()
{
  table_->removeAll();

  link_name_.clear();
  joint_name_.clear();
  role_.clear();
  cmd_iface_.clear();
  home_pos_.clear();
}

void JointConfigurationWidget::reset(int row)
{
  role_.at(row)->setCurrentText(kRoleLabel_Other);

  setDefaultValues(row);
  updateEnability(row);
}

void JointConfigurationWidget::setDefaultValues(int row)
{
  // 役割に応じてコマンドインターフェースとハードウェアインターフェースを設定
  switch (getRole(row)) {
    case tobas::jnt_role_t::LANDING_GEAR:
      cmd_iface_.at(row)->setCurrentText(kCmdIfaceLabel_Position);
      break;
    case tobas::jnt_role_t::PASSIVE_WHEEL:
      cmd_iface_.at(row)->setCurrentText(kCmdIfaceLabel_None);
      break;
    case tobas::jnt_role_t::MANIPULATION:
      cmd_iface_.at(row)->setCurrentText(kCmdIfaceLabel_Position);
      break;
    case tobas::jnt_role_t::OTHER:
      cmd_iface_.at(row)->setCurrentText(kCmdIfaceLabel_None);
      break;
    default:
      throw;
  }

  // 共通のデフォルト値
  home_pos_.at(row)->setValue(0);
}

void JointConfigurationWidget::updateEnability(int row)
{
  // 役割によるフィールド
  switch (getRole(row)) {
    case tobas::jnt_role_t::LANDING_GEAR:
      role_.at(row)->setEnabled(true);
      cmd_iface_.at(row)->setEnabled(true);
      home_pos_.at(row)->setEnabled(true);
      break;
    case tobas::jnt_role_t::PASSIVE_WHEEL:
      role_.at(row)->setEnabled(true);
      cmd_iface_.at(row)->setEnabled(false);
      home_pos_.at(row)->setEnabled(false);
      break;
    case tobas::jnt_role_t::MANIPULATION:
      role_.at(row)->setEnabled(true);
      cmd_iface_.at(row)->setEnabled(true);
      home_pos_.at(row)->setEnabled(true);
      break;
    case tobas::jnt_role_t::OTHER:
      role_.at(row)->setEnabled(true);
      cmd_iface_.at(row)->setEnabled(false);
      home_pos_.at(row)->setEnabled(false);
      break;
    default:
      throw;
  }
}

void JointConfigurationWidget::addLink(const std::string& link_name)
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
  TOBAS_CHECK(static_cast<size_t>(role->count()) == magic_enum::enum_count<tobas::jnt_role_t>());

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

  // Save each field
  link_name_.append(link_name_label);
  joint_name_.append(joint_name_label);
  role_.append(role);
  cmd_iface_.append(cmd_iface);
  home_pos_.append(home_pos);

  // Reset
  reset(row);

  // Connection
  connect(role, &qt::ComboBox::currentTextChanged, std::bind(&self::onRoleChanged, this, row));
}

void JointConfigurationWidget::onRoleChanged(int row)
{
  setDefaultValues(row);
  updateEnability(row);
}
}  // namespace sa
}  // namespace gui
