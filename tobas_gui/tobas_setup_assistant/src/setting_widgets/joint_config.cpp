#include <QLabel>
#include <QLineEdit>
#include <QDebug>
#include <QHeaderView>

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "tobas_setup_assistant/setting_tabs/joint_config.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace gui
{
namespace setup_assistant
{
JointConfigurationWidget::JointConfigurationWidget(const RobotInfo& robot) : robot_(robot)
{
  table_ = new qt::TableWidget(0, kNumCols);
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);  // 内容に合わせて横幅を自動調整
  table_->setHorizontalHeaderLabels({
    kLinkNameLabel,
    kJointNameLabel,
    kRoleLabel,
    kCmdIfaceLabel,
    kHwIfaceLabel,
    kChannelLabel,
    kHomePosLabel,
    kMinPosLabel,
    kMaxPosLabel,
    kMaxVelLabel,
    kMaxEffLabel,
  });

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

void JointConfigurationWidget::onOpened()
{
}

void JointConfigurationWidget::updateInternalDataStructures()
{
  table_->removeAll();

  for (const auto& [link_name, elem] : robot_.tree().getSegments())
  {
    if (link_name == robot_.tree().getRootName())
      continue;

    const auto& joint = elem.segment.joint();

    // 可動関節でなければスキップ
    if (joint.type == kdl::Joint::FIXED)
      continue;

    // テーブルに追加
    addLink(link_name);
  }
}

bool JointConfigurationWidget::isValid()
{
  // TODO: ハードウェアインターフェースが同じもののチャンネルが異なることを保証
  return true;
}

YAML::Node JointConfigurationWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  for (int row = 0; row < table_->rowCount(); ++row)
  {
    YAML::Node sub_node(YAML::NodeType::Map);
    sub_node[kRoleLabel] = role(row);
    sub_node[kCmdIfaceLabel] = commandInterface(row);
    sub_node[kHwIfaceLabel] = hardwareInterface(row);
    sub_node[kChannelLabel] = channel(row);
    sub_node[kHomePosLabel] = homePosition(row);

    node[linkName(row)] = sub_node;
  }

  return node;
}

void JointConfigurationWidget::load(const YAML::Node& node)
{
  for (const auto& pair : node)
  {
    const auto link_name = pair.first.as<QString>();
    const auto& sub_node = pair.second;

    const auto row = findLink(link_name);

    role(row, sub_node[kRoleLabel].as<tobas::jnt_role_t>());
    commandInterface(row, sub_node[kCmdIfaceLabel].as<tobas::jnt_cmd_iface_t>());
    hardwareInterface(row, sub_node[kHwIfaceLabel].as<tobas::jnt_hw_iface_t>());
    channel(row, sub_node[kChannelLabel].as<int>());
    homePosition(row, sub_node[kHomePosLabel].as<double>());
  }
}

QString JointConfigurationWidget::linkName(int row) const
{
  const auto cell = qobject_cast<QLabel*>(table_->cellWidget(row, kLinkNameCol));
  return cell->text();
}

QString JointConfigurationWidget::jointName(int row) const
{
  const auto cell = qobject_cast<QLabel*>(table_->cellWidget(row, kJointNameCol));
  return cell->text();
}

tobas::jnt_role_t JointConfigurationWidget::role(int row) const
{
  const auto cell = qobject_cast<qt::ComboBox*>(table_->cellWidget(row, kRoleCol));
  const auto text = cell->currentText();

  if (text == kRoleLabel_rotor)
    return tobas::jnt_role_t::ROTOR;
  else if (text == kRoleLabel_tilt)
    return tobas::jnt_role_t::TILT_JOINT;
  else if (text == kRoleLabel_cs)
    return tobas::jnt_role_t::CONTROL_SURFACE;
  else if (text == kRoleLabel_manip)
    return tobas::jnt_role_t::MANIPULATION;
  else if (text == kRoleLabel_wheel)
    return tobas::jnt_role_t::WHEEL;
  else if (text == kRoleLabel_other)
    return tobas::jnt_role_t::OTHER;
  else
    throw;
}

tobas::jnt_cmd_iface_t JointConfigurationWidget::commandInterface(int row) const
{
  const auto cell = qobject_cast<qt::ComboBox*>(table_->cellWidget(row, kCmdIfaceCol));
  const auto text = cell->currentText();

  if (text == kCmdIfaceLabel_pos)
    return tobas::jnt_cmd_iface_t::POSITION;
  else if (text == kCmdIfaceLabel_vel)
    return tobas::jnt_cmd_iface_t::VELOCITY;
  else if (text == kCmdIfaceLabel_eff)
    return tobas::jnt_cmd_iface_t::EFFORT;
  else
    throw;
}

tobas::jnt_hw_iface_t JointConfigurationWidget::hardwareInterface(int row) const
{
  const auto cell = qobject_cast<qt::ComboBox*>(table_->cellWidget(row, kHwIfaceCol));
  const auto text = cell->currentText();

  if (text == kHwIfaceLabel_pwm)
    return tobas::jnt_hw_iface_t::PWM;
  else if (text == kHwIfaceLabel_other)
    return tobas::jnt_hw_iface_t::OTHER;
  else
    throw;
}

int JointConfigurationWidget::channel(int row) const
{
  const auto cell = qobject_cast<qt::SpinBox*>(table_->cellWidget(row, kChannelCol));
  return cell->value();
}

double JointConfigurationWidget::homePosition(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, kHomePosCol));
  return cell->value();
}

void JointConfigurationWidget::role(int row, tobas::jnt_role_t value) const
{
  QString text;
  switch (value)
  {
    case tobas::jnt_role_t::ROTOR:
      text = kRoleLabel_rotor;
      break;
    case tobas::jnt_role_t::TILT_JOINT:
      text = kRoleLabel_tilt;
      break;
    case tobas::jnt_role_t::CONTROL_SURFACE:
      text = kRoleLabel_cs;
      break;
    case tobas::jnt_role_t::MANIPULATION:
      text = kRoleLabel_manip;
      break;
    case tobas::jnt_role_t::WHEEL:
      text = kRoleLabel_wheel;
      break;
    case tobas::jnt_role_t::OTHER:
      text = kRoleLabel_other;
      break;
    default:
      throw;
  }

  const auto cell = qobject_cast<qt::ComboBox*>(table_->cellWidget(row, kRoleCol));
  cell->setCurrentText(text);
}

void JointConfigurationWidget::commandInterface(int row, tobas::jnt_cmd_iface_t value) const
{
  QString text;
  switch (value)
  {
    case tobas::jnt_cmd_iface_t::POSITION:
      text = kCmdIfaceLabel_pos;
      break;
    case tobas::jnt_cmd_iface_t::VELOCITY:
      text = kCmdIfaceLabel_vel;
      break;
    case tobas::jnt_cmd_iface_t::EFFORT:
      text = kCmdIfaceLabel_eff;
      break;
    default:
      throw;
  }

  const auto cell = qobject_cast<qt::ComboBox*>(table_->cellWidget(row, kCmdIfaceCol));
  cell->setCurrentText(text);
}

void JointConfigurationWidget::hardwareInterface(int row, tobas::jnt_hw_iface_t value) const
{
  QString text;
  switch (value)
  {
    case tobas::jnt_hw_iface_t::PWM:
      text = kHwIfaceLabel_pwm;
      break;
    case tobas::jnt_hw_iface_t::OTHER:
      text = kHwIfaceLabel_other;
      break;
    default:
      throw;
  }

  const auto cell = qobject_cast<qt::ComboBox*>(table_->cellWidget(row, kHwIfaceCol));
  cell->setCurrentText(text);
}

void JointConfigurationWidget::channel(int row, int value) const
{
  const auto cell = qobject_cast<qt::SpinBox*>(table_->cellWidget(row, kChannelCol));
  cell->setValue(value);
}

void JointConfigurationWidget::homePosition(int row, double value) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, kHomePosCol));
  cell->setValue(value);
}

int JointConfigurationWidget::count() const
{
  return table_->rowCount();
}

int JointConfigurationWidget::findLink(const QString& link_name) const
{
  for (int row = 0; row < table_->rowCount(); ++row)
    if (linkName(row) == link_name)
      return row;

  qWarning() << "Link " << link_name << " is not found.";
  return -1;
}

int JointConfigurationWidget::findJoint(const QString& joint_name) const
{
  for (int row = 0; row < table_->rowCount(); ++row)
    if (jointName(row) == joint_name)
      return row;

  qWarning() << "Joint " << joint_name << " is not found.";
  return -1;
}

void JointConfigurationWidget::addLink(const std::string& link_name)
{
  const auto seg_it = robot_.tree().getSegment(link_name);
  const auto& joint = seg_it->second.segment.joint();

  // Name
  const auto link_name_label = new QLabel(QString::fromStdString(link_name));
  const auto joint_name_label = new QLabel(QString::fromStdString(joint.name));

  // Role
  const auto role = new qt::ComboBox();
  role->addItems({
    kRoleLabel_rotor,
    kRoleLabel_tilt,
    kRoleLabel_cs,
    kRoleLabel_manip,
    kRoleLabel_wheel,
    kRoleLabel_other,
  });
  role->setCurrentText(kRoleLabel_other);

  // Command Interface
  const auto cmd_iface = new qt::ComboBox();
  cmd_iface->addItems({ kCmdIfaceLabel_pos, kCmdIfaceLabel_vel, kCmdIfaceLabel_eff });
  cmd_iface->setCurrentText(kCmdIfaceLabel_pos);

  // Hardware Interface
  const auto hw_iface = new qt::ComboBox();
  hw_iface->addItems({ kHwIfaceLabel_pwm, kHwIfaceLabel_other });
  hw_iface->setCurrentText(kHwIfaceLabel_other);

  // Channel
  const auto channel = new qt::SpinBox();
  channel->setMinimum(0);
  channel->setValue(0);

  // Home Position
  const auto home_pos = new qt::DoubleSpinBox();
  home_pos->setDecimals(kPosDecimals);
  home_pos->setMinimum(joint.lower_limit);
  home_pos->setMaximum(joint.upper_limit);
  home_pos->setValue(0.);
  switch (joint.type)
  {
    case kdl::Joint::ROTATION:
      home_pos->setSuffix(" rad");
      break;
    case kdl::Joint::TRANSLATION:
      home_pos->setSuffix(" m");
      break;
    default:
      throw;
  }

  // Joint Limit (Read-only)
  const auto min_pos = new QLineEdit();
  const auto max_pos = new QLineEdit();
  const auto max_vel = new QLineEdit();
  const auto max_eff = new QLineEdit();
  min_pos->setReadOnly(true);
  max_pos->setReadOnly(true);
  max_vel->setReadOnly(true);
  max_eff->setReadOnly(true);
  switch (joint.type)
  {
    case kdl::Joint::ROTATION:
      min_pos->setText(QString::number(joint.lower_limit, 'f', kPosDecimals) + " rad");
      max_pos->setText(QString::number(joint.upper_limit, 'f', kPosDecimals) + " rad");
      max_vel->setText(QString::number(joint.max_velocity, 'f', kVelDecimals) + " rad/s");
      max_eff->setText(QString::number(joint.max_effort, 'f', kEffDecimals) + " Nm");
      break;
    case kdl::Joint::TRANSLATION:
      min_pos->setText(QString::number(joint.lower_limit, 'f', kPosDecimals) + " m");
      max_pos->setText(QString::number(joint.upper_limit, 'f', kPosDecimals) + " m");
      max_vel->setText(QString::number(joint.max_velocity, 'f', kVelDecimals) + " m/s");
      max_eff->setText(QString::number(joint.max_effort, 'f', kEffDecimals) + " N");
      break;
    default:
      throw;
  }

  // Insert table row
  const auto row = table_->rowCount();
  table_->insertRow(row);
  table_->setCellWidget(row, kLinkNameCol, link_name_label);
  table_->setCellWidget(row, kJointNameCol, joint_name_label);
  table_->setCellWidget(row, kRoleCol, role);
  table_->setCellWidget(row, kCmdIfaceCol, cmd_iface);
  table_->setCellWidget(row, kHwIfaceCol, hw_iface);
  table_->setCellWidget(row, kChannelCol, channel);
  table_->setCellWidget(row, kHomePosCol, home_pos);
  table_->setCellWidget(row, kMinPosCol, min_pos);
  table_->setCellWidget(row, kMaxPosCol, max_pos);
  table_->setCellWidget(row, kMaxVelCol, max_vel);
  table_->setCellWidget(row, kMaxEffCol, max_eff);
}
}  // namespace setup_assistant
}  // namespace gui
