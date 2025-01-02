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
JointConfigurationWidget::JointConfigurationWidget(
  const RobotInfo& robot,
  const propulsion_system::PropulsionSystemWidget* propulsion,
  const fixed_wing::FixedWingWidget* fixed_wing)
  : robot_(robot), propulsion_(propulsion), fixed_wing_(fixed_wing)
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
  // TODO: STM32CubeMXのようにPropulsionやFixed Wingが修正されたらシグナルスロットで即時反映

  const auto prop_link_names = propulsion_->selected()->linkNames();
  const auto tilt_joint_names = propulsion_->selected()->tiltJointNames();
  const auto cs_link_names = fixed_wing_->controlSurfaces()->selected()->linkNames();

  // 役割から開放されたジョイントをリセット
  for (int row = 0; row < table_->rowCount(); ++row)
  {
    const auto link_name = getLinkName(row);
    const auto joint_name = getJointName(row);

    switch (getRole(row))
    {
      case tobas::jnt_role_t::ROTOR:
        if (!prop_link_names.contains(link_name))
          resetRole(row);
        break;
      case tobas::jnt_role_t::TILT_JOINT:
        if (!tilt_joint_names.contains(joint_name))
          resetRole(row);
        break;
      case tobas::jnt_role_t::CONTROL_SURFACE:
        if (!cs_link_names.contains(link_name))
          resetRole(row);
        break;
      default:
        break;
    }
  }

  // 各役割を設定
  for (const auto& prop_link_name : prop_link_names)
  {
    const auto row = findLink(prop_link_name);
    role_[row]->setItemEnabled(kRoleLabel_rotor, true);
    setRole(row, tobas::jnt_role_t::ROTOR);
  }
  for (const auto& tilt_joint_name : tilt_joint_names)
  {
    const auto row = findJoint(tilt_joint_name);
    role_[row]->setItemEnabled(kRoleLabel_tilt, true);
    setRole(row, tobas::jnt_role_t::TILT_JOINT);
  }
  for (const auto& cs_link_name : cs_link_names)
  {
    const auto row = findLink(cs_link_name);
    role_[row]->setItemEnabled(kRoleLabel_cs, true);
    setRole(row, tobas::jnt_role_t::CONTROL_SURFACE);
  }
}

void JointConfigurationWidget::updateInternalDataStructures()
{
  clear();

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
  // ハードウェアインターフェースが同じもののチャンネルが異なることを保証
  QSet<int> pwm_channels;
  for (int row = 0; row < table_->rowCount(); ++row)
  {
    const auto channel = getChannel(row);
    switch (getHardwareInterface(row))
    {
      case tobas::jnt_hw_iface_t::PWM:
      {
        if (pwm_channels.contains(channel))
        {
          qt::qErrorBox(this, "PWM channel " + QString::number(channel) + " is duplicated.");
          return false;
        }
        pwm_channels.insert(channel);
        break;
      }
      case tobas::jnt_hw_iface_t::OTHER:
      {
        break;
      }
      default:
        throw;
    }
  }

  return true;
}

YAML::Node JointConfigurationWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  for (int row = 0; row < table_->rowCount(); ++row)
  {
    YAML::Node sub_node(YAML::NodeType::Map);
    sub_node[kRoleLabel] = getRole(row);
    sub_node[kCmdIfaceLabel] = getCommandInterface(row);
    sub_node[kHwIfaceLabel] = getHardwareInterface(row);
    sub_node[kChannelLabel] = getChannel(row);
    sub_node[kHomePosLabel] = getHomePosition(row);

    node[getLinkName(row)] = sub_node;
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

    setRole(row, sub_node[kRoleLabel].as<tobas::jnt_role_t>());
    setCommandInterface(row, sub_node[kCmdIfaceLabel].as<tobas::jnt_cmd_iface_t>());
    setHardwareInterface(row, sub_node[kHwIfaceLabel].as<tobas::jnt_hw_iface_t>());
    setChannel(row, sub_node[kChannelLabel].as<int>());
    setHomePosition(row, sub_node[kHomePosLabel].as<double>());
  }
}

QString JointConfigurationWidget::getLinkName(int row) const
{
  return link_name_[row]->text();
}

QString JointConfigurationWidget::getJointName(int row) const
{
  return joint_name_[row]->text();
}

tobas::jnt_role_t JointConfigurationWidget::getRole(int row) const
{
  const auto text = role_[row]->currentText();

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

tobas::jnt_cmd_iface_t JointConfigurationWidget::getCommandInterface(int row) const
{
  const auto text = cmd_iface_[row]->currentText();

  if (text == kCmdIfaceLabel_pos)
    return tobas::jnt_cmd_iface_t::POSITION;
  else if (text == kCmdIfaceLabel_vel)
    return tobas::jnt_cmd_iface_t::VELOCITY;
  else if (text == kCmdIfaceLabel_eff)
    return tobas::jnt_cmd_iface_t::EFFORT;
  else if (text == kCmdIfaceLabel_none)
    return tobas::jnt_cmd_iface_t::NONE;
  else
    throw;
}

tobas::jnt_hw_iface_t JointConfigurationWidget::getHardwareInterface(int row) const
{
  const auto text = hw_iface_[row]->currentText();

  if (text == kHwIfaceLabel_pwm)
    return tobas::jnt_hw_iface_t::PWM;
  else if (text == kHwIfaceLabel_other)
    return tobas::jnt_hw_iface_t::OTHER;
  else
    throw;
}

int JointConfigurationWidget::getChannel(int row) const
{
  return channel_[row]->value();
}

double JointConfigurationWidget::getHomePosition(int row) const
{
  return home_pos_[row]->value();
}

void JointConfigurationWidget::setRole(int row, tobas::jnt_role_t value)
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

  role_[row]->setCurrentText(text);
}

void JointConfigurationWidget::setCommandInterface(int row, tobas::jnt_cmd_iface_t value)
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
    case tobas::jnt_cmd_iface_t::NONE:
      text = kCmdIfaceLabel_none;
      break;
    default:
      throw;
  }

  cmd_iface_[row]->setCurrentText(text);
}

void JointConfigurationWidget::setHardwareInterface(int row, tobas::jnt_hw_iface_t value)
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

  hw_iface_[row]->setCurrentText(text);
}

void JointConfigurationWidget::setChannel(int row, int value)
{
  channel_[row]->setValue(value);
}

void JointConfigurationWidget::setHomePosition(int row, double value)
{
  home_pos_[row]->setValue(value);
}

int JointConfigurationWidget::count() const
{
  return table_->rowCount();
}

int JointConfigurationWidget::findLink(const QString& link_name) const
{
  for (int row = 0; row < table_->rowCount(); ++row)
    if (getLinkName(row) == link_name)
      return row;

  qWarning() << "Link " << link_name << " is not found.";
  return -1;
}

int JointConfigurationWidget::findJoint(const QString& joint_name) const
{
  for (int row = 0; row < table_->rowCount(); ++row)
    if (getJointName(row) == joint_name)
      return row;

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
  hw_iface_.clear();
  channel_.clear();
  home_pos_.clear();
  min_pos_.clear();
  max_pos_.clear();
  max_vel_.clear();
  max_eff_.clear();
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
  const auto getRole = new qt::ComboBox();
  getRole->addItems({
    kRoleLabel_rotor,
    kRoleLabel_tilt,
    kRoleLabel_cs,
    kRoleLabel_manip,
    kRoleLabel_wheel,
    kRoleLabel_other,
  });

  // Command Interface
  const auto cmd_iface = new qt::ComboBox();
  cmd_iface->addItems({
    kCmdIfaceLabel_pos,
    kCmdIfaceLabel_vel,
    kCmdIfaceLabel_eff,
    kCmdIfaceLabel_none,
  });

  // Hardware Interface
  const auto hw_iface = new qt::ComboBox();
  hw_iface->addItems({
    kHwIfaceLabel_pwm,
    kHwIfaceLabel_other,
  });

  // Channel
  const auto channel = new qt::SpinBox();
  channel->setMinimum(0);

  // Home Position
  const auto home_pos = new qt::DoubleSpinBox();
  home_pos->setDecimals(kPosDecimals);
  home_pos->setMinimum(joint.lower_limit);
  home_pos->setMaximum(joint.upper_limit);
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
  min_pos->setEnabled(false);
  max_pos->setEnabled(false);
  max_vel->setEnabled(false);
  max_eff->setEnabled(false);
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
  table_->insertRow(row);
  table_->setCellWidget(row, kLinkNameCol, link_name_label);
  table_->setCellWidget(row, kJointNameCol, joint_name_label);
  table_->setCellWidget(row, kRoleCol, getRole);
  table_->setCellWidget(row, kCmdIfaceCol, cmd_iface);
  table_->setCellWidget(row, kHwIfaceCol, hw_iface);
  table_->setCellWidget(row, kChannelCol, channel);
  table_->setCellWidget(row, kHomePosCol, home_pos);
  table_->setCellWidget(row, kMinPosCol, min_pos);
  table_->setCellWidget(row, kMaxPosCol, max_pos);
  table_->setCellWidget(row, kMaxVelCol, max_vel);
  table_->setCellWidget(row, kMaxEffCol, max_eff);

  // Save each field
  link_name_.append(link_name_label);
  joint_name_.append(joint_name_label);
  role_.append(getRole);
  cmd_iface_.append(cmd_iface);
  hw_iface_.append(hw_iface);
  channel_.append(channel);
  home_pos_.append(home_pos);
  min_pos_.append(min_pos);
  max_pos_.append(max_pos);
  max_vel_.append(max_vel);
  max_eff_.append(max_eff);

  // Reset
  resetRole(row);

  // Connection
  connect(getRole, &qt::ComboBox::currentTextChanged, std::bind(&self::onRoleChanged, this, row));
}

void JointConfigurationWidget::resetRole(int row)
{
  role_[row]->setCurrentText(kRoleLabel_other);
  onRoleChanged(row);

  // Rotor, Tilt Joint, Control Surfaceを選択不可にする
  for (const auto& label : { kRoleLabel_rotor, kRoleLabel_tilt, kRoleLabel_cs })
    role_[row]->setItemEnabled(label, false);
}

void JointConfigurationWidget::onRoleChanged(int row)
{
  switch (getRole(row))
  {
    case tobas::jnt_role_t::ROTOR:
      role_[row]->setEnabled(false);

      cmd_iface_[row]->setCurrentText(kCmdIfaceLabel_none);
      cmd_iface_[row]->setEnabled(false);

      hw_iface_[row]->setCurrentText(kHwIfaceLabel_other);
      hw_iface_[row]->setEnabled(false);

      channel_[row]->setValue(propulsion_->selected()->widget(getLinkName(row))->general()->channel());
      channel_[row]->setEnabled(false);

      home_pos_[row]->setValue(0.);
      home_pos_[row]->setEnabled(false);

      break;
    case tobas::jnt_role_t::TILT_JOINT:
      role_[row]->setEnabled(false);

      // 位置コマンドで固定
      cmd_iface_[row]->setCurrentText(kCmdIfaceLabel_pos);
      cmd_iface_[row]->setEnabled(false);

      // PWMがデフォルトだが変更も可能
      hw_iface_[row]->setCurrentText(kHwIfaceLabel_pwm);
      hw_iface_[row]->setEnabled(true);

      channel_[row]->setEnabled(true);

      home_pos_[row]->setValue(0.);
      home_pos_[row]->setEnabled(false);

      break;
    case tobas::jnt_role_t::CONTROL_SURFACE:
      role_[row]->setEnabled(false);

      // 位置コマンドで固定
      cmd_iface_[row]->setCurrentText(kCmdIfaceLabel_pos);
      cmd_iface_[row]->setEnabled(false);

      // PWMがデフォルトだが変更も可能
      hw_iface_[row]->setCurrentText(kHwIfaceLabel_pwm);
      hw_iface_[row]->setEnabled(true);

      channel_[row]->setEnabled(true);

      home_pos_[row]->setValue(0.);
      home_pos_[row]->setEnabled(false);

      break;
    case tobas::jnt_role_t::MANIPULATION:
      role_[row]->setEnabled(true);

      cmd_iface_[row]->setEnabled(true);

      hw_iface_[row]->setEnabled(true);

      channel_[row]->setEnabled(true);

      home_pos_[row]->setEnabled(true);

      break;
    case tobas::jnt_role_t::WHEEL:
      role_[row]->setEnabled(true);

      cmd_iface_[row]->setCurrentText(kCmdIfaceLabel_none);
      cmd_iface_[row]->setEnabled(false);

      hw_iface_[row]->setCurrentText(kHwIfaceLabel_other);
      hw_iface_[row]->setEnabled(false);

      channel_[row]->setValue(0);
      channel_[row]->setEnabled(false);

      home_pos_[row]->setValue(0.);
      home_pos_[row]->setEnabled(false);

      break;
    case tobas::jnt_role_t::OTHER:
      role_[row]->setEnabled(true);

      cmd_iface_[row]->setCurrentText(kCmdIfaceLabel_none);
      cmd_iface_[row]->setEnabled(false);

      hw_iface_[row]->setCurrentText(kHwIfaceLabel_other);
      hw_iface_[row]->setEnabled(false);

      channel_[row]->setValue(0);
      channel_[row]->setEnabled(false);

      home_pos_[row]->setValue(0.);
      home_pos_[row]->setEnabled(false);

      break;
    default:
      throw;
  }
}
}  // namespace setup_assistant
}  // namespace gui
