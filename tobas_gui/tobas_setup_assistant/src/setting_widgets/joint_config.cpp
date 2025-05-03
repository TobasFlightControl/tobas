#include <QHeaderView>
#include <QDebug>

#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "tobas_setup_assistant/setting_tabs/joint_config.hpp"
#include "tobas_setup_assistant/constants.hpp"

namespace gui
{
namespace sa
{
JointConfigurationWidget::JointConfigurationWidget(
  const RobotInfo& robot,
  const Signals& _signals,
  const propulsion::PropulsionSystemWidget* propulsion,
  const fixed_wing::FixedWingWidget* fixed_wing)
  : robot_(robot), signals_(_signals), propulsion_(propulsion), fixed_wing_(fixed_wing)
{
  table_ = new qt::TableWidget(0, kNumCols);
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);  // 内容に合わせて横幅を自動調整
  table_->setHorizontalHeaderLabels({
    kLinkNameLabel,
    kJointNameLabel,
    kRoleLabel,
    kCmdIfaceLabel,
    kHwIfaceLabel,
    kHomePosLabel,
    kPwmChannelLabel,
    kPwmMinPeriodLabel,
    kPwmMaxPeriodLabel,
    kPwmMinAngleLabel,
    kPwmMaxAngleLabel,
    kPwmReverseLabel,
  });
  addWidget(table_);

  connect(&signals_, &Signals::rotorLinkAdded, this, &self::onRotorLinkAdded);
  connect(&signals_, &Signals::rotorLinkRemoved, this, &self::onRotorLinkRemoved);
  connect(&signals_, &Signals::isTiltRotorStateChanged, this, &self::onIsTiltRotorStateChanged);
  connect(&signals_, &Signals::tiltJointNameChanged, this, &self::onTiltJointNameChanged);

  const auto css = fixed_wing_->controlSurfaces();
  connect(css, &fixed_wing::ControlSurfacesWidget::linkAdded, this, &self::onControlSurfaceLinkAdded);
  connect(css, &fixed_wing::ControlSurfacesWidget::linkRemoved, this, &self::onControlSurfaceLinkRemoved);
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

    // テーブルに追加
    addLink(link_name);
  }
}

bool JointConfigurationWidget::isValid()
{
  QSet<int> pwm_channels;

  for (int row = 0; row < table_->rowCount(); ++row) {
    // ハードウェアインターフェースが同じもののチャンネルが異なることを保証
    const auto channel = getPwmChannel(row);
    switch (getHardwareInterface(row)) {
      case tobas::hw_iface_t::PWM: {
        if (pwm_channels.contains(channel)) {
          qt::qErrorBox(this, "PWM channel " + QString::number(channel) + " is duplicated.");
          return false;
        }
        pwm_channels.insert(channel);
        break;
      }
      case tobas::hw_iface_t::OTHER: {
        break;
      }
      default:
        throw;
    }

    // PWM High時間の範囲
    if (getPwmMinPeriod(row) >= getPwmMaxPeriod(row)) {
      qt::qErrorBox(this, "PWM period range of channel " + QString::number(channel) + " is invalid.");
      return false;
    }

    // PWM関節角の範囲
    if (getPwmMinAngle(row) >= getPwmMaxAngle(row)) {
      qt::qErrorBox(this, "PWM angle range of channel " + QString::number(channel) + " is invalid.");
      return false;
    }
  }

  return true;
}

YAML::Node JointConfigurationWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int row = 0; row < table_->rowCount(); ++row) {
    YAML::Node sub_node(YAML::NodeType::Map);
    sub_node[kRoleLabel] = role_[row]->currentText();
    sub_node[kCmdIfaceLabel] = cmd_iface_[row]->currentText();
    sub_node[kHwIfaceLabel] = hw_iface_[row]->currentText();
    sub_node[kHomePosLabel] = home_pos_[row]->value();
    sub_node[kPwmChannelLabel] = pwm_channel_[row]->value();
    sub_node[kPwmMinPeriodLabel] = pwm_min_period_[row]->value();
    sub_node[kPwmMaxPeriodLabel] = pwm_max_period_[row]->value();
    sub_node[kPwmMinAngleLabel] = pwm_min_angle_[row]->value();
    sub_node[kPwmMaxAngleLabel] = pwm_max_angle_[row]->value();
    sub_node[kPwmReverseLabel] = pwm_reverse_[row]->isChecked();

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

    role_[row]->setCurrentText(sub_node[kRoleLabel].as<QString>());
    cmd_iface_[row]->setCurrentText(sub_node[kCmdIfaceLabel].as<QString>());
    hw_iface_[row]->setCurrentText(sub_node[kHwIfaceLabel].as<QString>());
    home_pos_[row]->setValue(sub_node[kHomePosLabel].as<int>());
    pwm_channel_[row]->setValue(sub_node[kPwmChannelLabel].as<int>());
    pwm_min_period_[row]->setValue(sub_node[kPwmMinPeriodLabel].as<int>());
    pwm_max_period_[row]->setValue(sub_node[kPwmMaxPeriodLabel].as<int>());
    pwm_min_angle_[row]->setValue(sub_node[kPwmMinAngleLabel].as<int>());
    pwm_max_angle_[row]->setValue(sub_node[kPwmMaxAngleLabel].as<int>());
    pwm_reverse_[row]->setChecked(sub_node[kPwmReverseLabel].as<bool>());

    updateEnability(row);
  }

  // ロータに対応するティルトジョイント名を更新
  tilt_joint_map_.clear();
  for (int i = 0; i < propulsion_->numUnits(); ++i) {
    if (propulsion_->isTiltRotor(i)) {
      tilt_joint_map_[propulsion_->linkName(i)] = propulsion_->tiltJointName(i);
    }
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

  if (text == kRoleLabel_Rotor) {
    return tobas::jnt_role_t::ROTOR;
  }
  else if (text == kRoleLabel_TiltJoint) {
    return tobas::jnt_role_t::TILT_JOINT;
  }
  else if (text == kRoleLabel_ControlSurface) {
    return tobas::jnt_role_t::CONTROL_SURFACE;
  }
  else if (text == kRoleLabel_Manipulation) {
    return tobas::jnt_role_t::MANIPULATION;
  }
  else if (text == kRoleLabel_PassiveWheel) {
    return tobas::jnt_role_t::PASSIVE_WHEEL;
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
  const auto text = cmd_iface_[row]->currentText();

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

tobas::hw_iface_t JointConfigurationWidget::getHardwareInterface(int row) const
{
  const auto text = hw_iface_[row]->currentText();

  if (text == kHwIfaceLabel_PWM) {
    return tobas::hw_iface_t::PWM;
  }
  else if (text == kHwIfaceLabel_Other) {
    return tobas::hw_iface_t::OTHER;
  }
  else {
    throw;
  }
}

double JointConfigurationWidget::getHomePosition(int row) const
{
  return tobas_std::deg2rad(home_pos_[row]->value());
}

int JointConfigurationWidget::getPwmChannel(int row) const
{
  return pwm_channel_[row]->value();
}

uint16_t JointConfigurationWidget::getPwmMinPeriod(int row) const
{
  return pwm_min_period_[row]->value();
}

uint16_t JointConfigurationWidget::getPwmMaxPeriod(int row) const
{
  return pwm_max_period_[row]->value();
}

double JointConfigurationWidget::getPwmMinAngle(int row) const
{
  return tobas_std::deg2rad(pwm_min_angle_[row]->value());
}

double JointConfigurationWidget::getPwmMaxAngle(int row) const
{
  return tobas_std::deg2rad(pwm_max_angle_[row]->value());
}

bool JointConfigurationWidget::getPwmReverse(int row) const
{
  return pwm_reverse_[row]->isChecked();
}

void JointConfigurationWidget::setRole(int row, tobas::jnt_role_t value)
{
  QString text;
  switch (value) {
    case tobas::jnt_role_t::ROTOR:
      text = kRoleLabel_Rotor;
      break;
    case tobas::jnt_role_t::TILT_JOINT:
      text = kRoleLabel_TiltJoint;
      break;
    case tobas::jnt_role_t::CONTROL_SURFACE:
      text = kRoleLabel_ControlSurface;
      break;
    case tobas::jnt_role_t::MANIPULATION:
      text = kRoleLabel_Manipulation;
      break;
    case tobas::jnt_role_t::PASSIVE_WHEEL:
      text = kRoleLabel_PassiveWheel;
      break;
    case tobas::jnt_role_t::OTHER:
      text = kRoleLabel_Other;
      break;
    default:
      throw;
  }

  role_[row]->setCurrentText(text);
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

  cmd_iface_[row]->setCurrentText(text);
}

void JointConfigurationWidget::setHardwareInterface(int row, tobas::hw_iface_t value)
{
  QString text;
  switch (value) {
    case tobas::hw_iface_t::PWM:
      text = kHwIfaceLabel_PWM;
      break;
    case tobas::hw_iface_t::OTHER:
      text = kHwIfaceLabel_Other;
      break;
    default:
      throw;
  }

  hw_iface_[row]->setCurrentText(text);
}

void JointConfigurationWidget::setHomePosition(int row, double value)
{
  home_pos_[row]->setValue(std::round(tobas_std::rad2deg(value)));
}

void JointConfigurationWidget::setPwmChannel(int row, int value)
{
  pwm_channel_[row]->setValue(value);
}

void JointConfigurationWidget::setPwmMinPeriod(int row, uint16_t value)
{
  pwm_min_period_[row]->setValue(value);
}

void JointConfigurationWidget::setPwmMaxPeriod(int row, uint16_t value)
{
  pwm_max_period_[row]->setValue(value);
}

void JointConfigurationWidget::setPwmMinAngle(int row, double value)
{
  pwm_min_angle_[row]->setValue(std::round(tobas_std::rad2deg(value)));
}

void JointConfigurationWidget::setPwmMaxAngle(int row, double value)
{
  pwm_max_angle_[row]->setValue(std::round(tobas_std::rad2deg(value)));
}

void JointConfigurationWidget::setPwmReverse(int row, bool value)
{
  pwm_reverse_[row]->setChecked(value);
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
  hw_iface_.clear();
  home_pos_.clear();
  pwm_channel_.clear();
  pwm_min_period_.clear();
  pwm_max_period_.clear();
  pwm_min_angle_.clear();
  pwm_max_angle_.clear();
  pwm_reverse_.clear();

  tilt_joint_map_.clear();
}

void JointConfigurationWidget::reset(int row)
{
  role_[row]->setCurrentText(kRoleLabel_Other);

  // Rotor, Tilt Joint, Control Surfaceを選択不可にする
  for (const auto& label : { kRoleLabel_Rotor, kRoleLabel_TiltJoint, kRoleLabel_ControlSurface }) {
    role_[row]->setItemEnabled(label, false);
  }

  setDefaultValues(row);
  updateEnability(row);
}

void JointConfigurationWidget::setDefaultValues(int row)
{
  // 役割に応じてコマンドインターフェースとハードウェアインターフェースを設定
  switch (getRole(row)) {
    case tobas::jnt_role_t::ROTOR:
      cmd_iface_[row]->setCurrentText(kCmdIfaceLabel_None);
      hw_iface_[row]->setCurrentText(kHwIfaceLabel_Other);
      break;
    case tobas::jnt_role_t::TILT_JOINT:
      cmd_iface_[row]->setCurrentText(kCmdIfaceLabel_Position);  // 位置コマンドで固定
      hw_iface_[row]->setCurrentText(kHwIfaceLabel_PWM);
      break;
    case tobas::jnt_role_t::CONTROL_SURFACE:
      cmd_iface_[row]->setCurrentText(kCmdIfaceLabel_Position);  // 位置コマンドで固定
      hw_iface_[row]->setCurrentText(kHwIfaceLabel_PWM);
      break;
    case tobas::jnt_role_t::MANIPULATION:
      cmd_iface_[row]->setCurrentText(kCmdIfaceLabel_Position);
      hw_iface_[row]->setCurrentText(kHwIfaceLabel_PWM);
      break;
    case tobas::jnt_role_t::PASSIVE_WHEEL:
      cmd_iface_[row]->setCurrentText(kCmdIfaceLabel_None);
      hw_iface_[row]->setCurrentText(kHwIfaceLabel_Other);
      break;
    case tobas::jnt_role_t::OTHER:
      cmd_iface_[row]->setCurrentText(kCmdIfaceLabel_None);
      hw_iface_[row]->setCurrentText(kHwIfaceLabel_Other);
      break;
    default:
      throw;
  }

  // 共通のデフォルト値
  home_pos_[row]->setValue(0);
  pwm_channel_[row]->setValue(0);
  pwm_min_period_[row]->setValue(1000);
  pwm_max_period_[row]->setValue(2000);
  pwm_min_angle_[row]->setValue(-90);
  pwm_max_angle_[row]->setValue(90);
}

void JointConfigurationWidget::updateEnability(int row)
{
  // 役割によるフィールド
  switch (getRole(row)) {
    case tobas::jnt_role_t::ROTOR:
      role_[row]->setEnabled(false);
      cmd_iface_[row]->setEnabled(false);
      hw_iface_[row]->setEnabled(false);
      home_pos_[row]->setEnabled(false);
      break;
    case tobas::jnt_role_t::TILT_JOINT:
      role_[row]->setEnabled(false);
      cmd_iface_[row]->setEnabled(false);
      hw_iface_[row]->setEnabled(true);
      home_pos_[row]->setEnabled(false);
      break;
    case tobas::jnt_role_t::CONTROL_SURFACE:
      role_[row]->setEnabled(false);
      cmd_iface_[row]->setEnabled(false);
      hw_iface_[row]->setEnabled(true);
      home_pos_[row]->setEnabled(false);
      break;
    case tobas::jnt_role_t::MANIPULATION:
      role_[row]->setEnabled(true);
      cmd_iface_[row]->setEnabled(true);
      hw_iface_[row]->setEnabled(true);
      home_pos_[row]->setEnabled(true);
      break;
    case tobas::jnt_role_t::PASSIVE_WHEEL:
      role_[row]->setEnabled(true);
      cmd_iface_[row]->setEnabled(false);
      hw_iface_[row]->setEnabled(false);
      home_pos_[row]->setEnabled(false);
      break;
    case tobas::jnt_role_t::OTHER:
      role_[row]->setEnabled(true);
      cmd_iface_[row]->setEnabled(false);
      hw_iface_[row]->setEnabled(false);
      home_pos_[row]->setEnabled(false);
      break;
    default:
      throw;
  }

  // ハードウェアインターフェースによるフィールド
  switch (getHardwareInterface(row)) {
    case tobas::hw_iface_t::PWM:
      pwm_channel_[row]->setEnabled(true);
      pwm_min_period_[row]->setEnabled(true);
      pwm_max_period_[row]->setEnabled(true);
      pwm_min_angle_[row]->setEnabled(true);
      pwm_max_angle_[row]->setEnabled(true);
      pwm_reverse_[row]->setEnabled(true);
      break;
    case tobas::hw_iface_t::OTHER:
      pwm_channel_[row]->setEnabled(false);
      pwm_min_period_[row]->setEnabled(false);
      pwm_max_period_[row]->setEnabled(false);
      pwm_min_angle_[row]->setEnabled(false);
      pwm_max_angle_[row]->setEnabled(false);
      pwm_reverse_[row]->setEnabled(false);
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
    kRoleLabel_Rotor,
    kRoleLabel_TiltJoint,
    kRoleLabel_ControlSurface,
    kRoleLabel_Manipulation,
    kRoleLabel_PassiveWheel,
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

  // Hardware Interface
  const auto hw_iface = new qt::ComboBox();
  hw_iface->addItems({
    kHwIfaceLabel_PWM,
    kHwIfaceLabel_Other,
  });

  // Channel
  const auto pwm_channel = new qt::SpinBox();
  pwm_channel->setMinimum(0);

  // Home Position
  const auto home_pos = new qt::SpinBox();
  home_pos->setMinimum(std::round(tobas_std::rad2deg(std::isinf(joint.lower_limit) ? -M_PI : joint.lower_limit)));
  home_pos->setMaximum(std::round(tobas_std::rad2deg(std::isinf(joint.upper_limit) ? M_PI : joint.upper_limit)));
  home_pos->setSuffix(" deg");

  // PWM Period Range
  const auto pwm_min_period = new qt::SpinBox();
  const auto pwm_max_period = new qt::SpinBox();
  pwm_min_period->setMinimum(0);
  pwm_max_period->setMinimum(0);
  pwm_min_period->setMaximum(2500);
  pwm_max_period->setMaximum(2500);
  pwm_min_period->setSuffix(" us");
  pwm_max_period->setSuffix(" us");

  // PWM Joint Angle Range
  const auto pwm_min_angle = new qt::SpinBox();
  const auto pwm_max_angle = new qt::SpinBox();
  pwm_min_angle->setMinimum(-180);
  pwm_min_angle->setMaximum(0);
  pwm_max_angle->setMinimum(0);
  pwm_max_angle->setMaximum(180);
  pwm_min_angle->setSuffix(" deg");
  pwm_max_angle->setSuffix(" deg");

  // PWM Reverse
  const auto pwm_reverse = new QPushButton();
  pwm_reverse->setCheckable(true);
  pwm_reverse->setText(kReverseLabel_Normal);
  connect(
    pwm_reverse,
    &QPushButton::toggled,
    this,
    [pwm_reverse](bool checked)
    {
      if (checked) {
        pwm_reverse->setText(kReverseLabel_Reverse);
      }
      else {
        pwm_reverse->setText(kReverseLabel_Normal);
      }
    });

  // Insert table row
  table_->insertRow(row);
  table_->setCellWidget(row, kLinkNameCol, link_name_label);
  table_->setCellWidget(row, kJointNameCol, joint_name_label);
  table_->setCellWidget(row, kRoleCol, role);
  table_->setCellWidget(row, kCmdIfaceCol, cmd_iface);
  table_->setCellWidget(row, kHwIfaceCol, hw_iface);
  table_->setCellWidget(row, kPwmChannelCol, pwm_channel);
  table_->setCellWidget(row, kHomePosCol, home_pos);
  table_->setCellWidget(row, kPwmMinPeriodCol, pwm_min_period);
  table_->setCellWidget(row, kPwmMaxPeriodCol, pwm_max_period);
  table_->setCellWidget(row, kPwmMinAngleCol, pwm_min_angle);
  table_->setCellWidget(row, kPwmMaxAngleCol, pwm_max_angle);
  table_->setCellWidget(row, kPwmReverseCol, pwm_reverse);

  // Save each field
  link_name_.append(link_name_label);
  joint_name_.append(joint_name_label);
  role_.append(role);
  cmd_iface_.append(cmd_iface);
  hw_iface_.append(hw_iface);
  home_pos_.append(home_pos);
  pwm_channel_.append(pwm_channel);
  pwm_min_period_.append(pwm_min_period);
  pwm_max_period_.append(pwm_max_period);
  pwm_min_angle_.append(pwm_min_angle);
  pwm_max_angle_.append(pwm_max_angle);
  pwm_reverse_.append(pwm_reverse);

  // Reset
  reset(row);

  // Connection
  connect(role, &qt::ComboBox::currentTextChanged, std::bind(&self::onRoleChanged, this, row));
  connect(hw_iface, &qt::ComboBox::currentTextChanged, std::bind(&self::onHardwareInterfaceChanged, this, row));
}

void JointConfigurationWidget::removeTiltJoint(const QString& rotor_link_name)
{
  TOBAS_CHECK(tilt_joint_map_.contains(rotor_link_name));
  const auto& tilt_joint_name = tilt_joint_map_[rotor_link_name];

  const auto tilt_row = findJoint(tilt_joint_name);
  TOBAS_CHECK(tilt_row >= 0);

  const auto tilt_role = getRole(tilt_row);
  TOBAS_CHECK(tilt_role == tobas::jnt_role_t::TILT_JOINT);

  tilt_joint_map_.remove(rotor_link_name);
  reset(tilt_row);
}

void JointConfigurationWidget::onRoleChanged(int row)
{
  setDefaultValues(row);
  updateEnability(row);
}

void JointConfigurationWidget::onHardwareInterfaceChanged(int row)
{
  updateEnability(row);
}

void JointConfigurationWidget::onRotorLinkAdded(const QString& link_name)
{
  const auto row = findLink(link_name);
  TOBAS_CHECK(row >= 0);

  // TODO: そもそもPropulsionSystemやFixedWing側でリンクが被らないようにする
  const auto cur_role = getRole(row);
  TOBAS_CHECK(cur_role != tobas::jnt_role_t::TILT_JOINT);
  TOBAS_CHECK(cur_role != tobas::jnt_role_t::CONTROL_SURFACE);

  role_[row]->setItemEnabled(kRoleLabel_Rotor, true);
  setRole(row, tobas::jnt_role_t::ROTOR);
}

void JointConfigurationWidget::onRotorLinkRemoved(const QString& link_name)
{
  const auto row = findLink(link_name);
  TOBAS_CHECK(row >= 0);

  const auto cur_role = getRole(row);
  TOBAS_CHECK(cur_role == tobas::jnt_role_t::ROTOR);

  if (tilt_joint_map_.contains(link_name)) {
    removeTiltJoint(link_name);
  }

  reset(row);
}

void JointConfigurationWidget::onIsTiltRotorStateChanged(const QString& link_name, bool is_tilt)
{
  const auto rotor_row = findLink(link_name);
  TOBAS_CHECK(rotor_row >= 0);

  const auto rotor_role = getRole(rotor_row);
  TOBAS_CHECK(rotor_role == tobas::jnt_role_t::ROTOR);

  if (!is_tilt) {
    removeTiltJoint(link_name);
  }
}

void JointConfigurationWidget::onTiltJointNameChanged(const QString& link_name, const QString& tilt_joint_name)
{
  const auto rotor_row = findLink(link_name);
  TOBAS_CHECK(rotor_row >= 0);

  const auto rotor_role = getRole(rotor_row);
  TOBAS_CHECK(rotor_role == tobas::jnt_role_t::ROTOR);

  if (tilt_joint_map_.contains(link_name)) {
    removeTiltJoint(tilt_joint_name);
  }
  tilt_joint_map_[link_name] = tilt_joint_name;

  const auto tilt_row = findJoint(tilt_joint_name);
  TOBAS_CHECK(tilt_row >= 0);

  const auto tilt_role = getRole(tilt_row);
  TOBAS_CHECK(tilt_role != tobas::jnt_role_t::ROTOR);
  TOBAS_CHECK(tilt_role != tobas::jnt_role_t::CONTROL_SURFACE);

  role_[tilt_row]->setItemEnabled(kRoleLabel_TiltJoint, true);
  setRole(tilt_row, tobas::jnt_role_t::TILT_JOINT);
}

void JointConfigurationWidget::onControlSurfaceLinkAdded(const QString& link_name)
{
  const auto row = findLink(link_name);
  TOBAS_CHECK(row >= 0);

  const auto cur_role = getRole(row);
  TOBAS_CHECK(cur_role != tobas::jnt_role_t::ROTOR);
  TOBAS_CHECK(cur_role != tobas::jnt_role_t::TILT_JOINT);

  role_[row]->setItemEnabled(kRoleLabel_ControlSurface, true);
  setRole(row, tobas::jnt_role_t::CONTROL_SURFACE);
}

void JointConfigurationWidget::onControlSurfaceLinkRemoved(const QString& link_name)
{
  const auto row = findLink(link_name);
  TOBAS_CHECK(row >= 0);

  const auto cur_role = getRole(row);
  TOBAS_CHECK(cur_role == tobas::jnt_role_t::CONTROL_SURFACE);

  reset(row);
}
}  // namespace sa
}  // namespace gui
