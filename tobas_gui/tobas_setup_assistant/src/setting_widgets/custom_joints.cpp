#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/custom_joints.hpp"

namespace gui
{
namespace setup_assistant
{
CustomJointsWidget::CustomJointsWidget(const RobotInfo& robot) : robot_(robot)
{
}

const char* CustomJointsWidget::name() const
{
  return "Custom Joints";
}

const char* CustomJointsWidget::title() const
{
  return "Define Custom Joints";
}

const char* CustomJointsWidget::description() const
{
  return "Configure the settings for joints with transmissions "
         "other than those in the propulsion system and fixed-wing control surfaces.";
}

void CustomJointsWidget::onInit()
{
  const QStringList labels{ kLinkNameLabel, kJointNameLabel, kHomePosLabel, kMinPosLabel, kMaxPosLabel,
                            kCmdTypeLabel,  kPGainLabel,     kIGainLabel,   kDGainLabel };
  table_ = new qt::TableWidget(0, labels.size());
  table_->setHorizontalHeaderLabels(labels);
  for (int c = 0; c < table_->columnCount(); ++c)
    table_->setColumnWidth(c, kColWidth);
  addWidget(table_);
}

void CustomJointsWidget::onOpened()
{
  return;
}

void CustomJointsWidget::updateInternalDataStructures()
{
  table_->removeAll();
  int row = 0;

  for (const auto& [link_name, elem] : robot_.tree().getSegments())
  {
    const auto& joint = elem.segment.joint();

    // 可動ジョイントでない場合はスキップ
    if (joint.type == kdl::Joint::Fixed)
      continue;

    // トランスミッションが無効な場合はスキップ
    const auto hi = robot_.hardwareInterface(joint.name);
    if (hi < 0)
      return;

    table_->insertRow(row);

    const auto link_name_label = new QLabel(QString::fromStdString(link_name));
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
        qt::qErrorBox(this, QString::fromStdString(std::format("Unexpected joint type: {}", (int)joint.type)));
        continue;
    }

    const auto cmd_type = new qt::ComboBox();

    const auto p_gain = new qt::DoubleSpinBox();
    const auto i_gain = new qt::DoubleSpinBox();
    const auto d_gain = new qt::DoubleSpinBox();

    p_gain->setDecimals(kGainDecimals);
    i_gain->setDecimals(kGainDecimals);
    d_gain->setDecimals(kGainDecimals);

    p_gain->setValue(kDefaultPGain);
    i_gain->setValue(kDefaultIGain);
    d_gain->setValue(kDefaultDGain);

    switch (hi)
    {
      case hw_interface::POSITION:
        cmd_type->addItem(kPositionLabel);
        p_gain->setEnabled(false);
        i_gain->setEnabled(false);
        d_gain->setEnabled(false);
        break;
      case hw_interface::VELOCITY:
        cmd_type->addItem(kPositionLabel);
        cmd_type->addItem(kVelocityLabel);
        cmd_type->setCurrentText(kVelocityLabel);
        break;
      case hw_interface::EFFORT:
        cmd_type->addItem(kPositionLabel);
        cmd_type->addItem(kVelocityLabel);
        cmd_type->addItem(kEffortLabel);
        cmd_type->setCurrentText(kEffortLabel);
        break;
      default:
        qt::qErrorBox(this, QString::fromStdString(std::format("Unknown hardware interface: {}", (int)hi)));
        continue;
    }

    table_->setCellWidget(row, static_cast<int>(LINK_NAME), link_name_label);
    table_->setCellWidget(row, static_cast<int>(JOINT_NAME), jnt_name_label);
    table_->setCellWidget(row, static_cast<int>(HOME_POSITION), home_pos);
    table_->setCellWidget(row, static_cast<int>(MIN_POSITION), min_pos);
    table_->setCellWidget(row, static_cast<int>(MAX_POSITION), max_pos);
    table_->setCellWidget(row, static_cast<int>(COMMAND_TYPE), cmd_type);
    table_->setCellWidget(row, static_cast<int>(P_GAIN), p_gain);
    table_->setCellWidget(row, static_cast<int>(I_GAIN), i_gain);
    table_->setCellWidget(row, static_cast<int>(D_GAIN), d_gain);

    ++row;
  }
}

bool CustomJointsWidget::isValid()
{
  for (int row = 0; row < count(); ++row)
  {
    const auto jnt_name = getJointName(row);
    const auto home_pos = getHomePosition(row);
    const auto min_pos = getMinPosition(row);
    const auto max_pos = getMaxPosition(row);

    if (min_pos > max_pos)
    {
      qt::qErrorBox(this, "Position limit of joint \"" + jnt_name + "\" is invalid.");
      return false;
    }
    if (home_pos < min_pos || max_pos < home_pos)
    {
      qt::qErrorBox(this, "Home position of joint \"" + jnt_name + "\" is out of its limit.");
      return false;
    }
  }

  return true;
}

YAML::Node CustomJointsWidget::dump()
{
  YAML::Node node(YAML::NodeType::Sequence);

  for (int row = 0; row < count(); ++row)
  {
    YAML::Node jnt_node(YAML::NodeType::Map);
    jnt_node[kJointNameLabel] = getJointName(row);
    jnt_node[kHomePosLabel] = getHomePosition(row);
    jnt_node[kMinPosLabel] = getMinPosition(row);
    jnt_node[kMaxPosLabel] = getMaxPosition(row);
    jnt_node[kCmdTypeLabel] = getCommandType(row);
    jnt_node[kPGainLabel] = getPGain(row);
    jnt_node[kIGainLabel] = getIGain(row);
    jnt_node[kDGainLabel] = getDGain(row);
    node.push_back(jnt_node);
  }

  return node;
}

void CustomJointsWidget::load(const YAML::Node& node)
{
  for (const auto& jnt_node : node)
  {
    const auto jnt_name = jnt_node[kJointNameLabel].as<QString>();
    const auto row = getRow(jnt_name);
    if (row < 0)
      qt::qErrorBox(this, "\"" + jnt_name + "\" does not exist in the custom joint list.");
    continue;

    setHomePosition(row, jnt_node[kHomePosLabel].as<double>());
    setMinPosition(row, jnt_node[kMinPosLabel].as<double>());
    setMaxPosition(row, jnt_node[kMaxPosLabel].as<double>());
    setCommandType(row, jnt_node[kCmdTypeLabel].as<tobas::joint_control_type_t>());
    setPGain(row, jnt_node[kPGainLabel].as<double>());
    setIGain(row, jnt_node[kIGainLabel].as<double>());
    setDGain(row, jnt_node[kDGainLabel].as<double>());
  }
}

int CustomJointsWidget::count() const
{
  return table_->rowCount();
}

QString CustomJointsWidget::getLinkName(int row) const
{
  const auto cell = qobject_cast<QLabel*>(table_->cellWidget(row, static_cast<int>(LINK_NAME)));
  return cell->text();
}

void CustomJointsWidget::setLinkName(int row, const QString& text)
{
  const auto cell = qobject_cast<QLabel*>(table_->cellWidget(row, static_cast<int>(LINK_NAME)));
  cell->setText(text);
}

QString CustomJointsWidget::getJointName(int row) const
{
  const auto cell = qobject_cast<QLabel*>(table_->cellWidget(row, static_cast<int>(JOINT_NAME)));
  return cell->text();
}

void CustomJointsWidget::setJointName(int row, const QString& text)
{
  const auto cell = qobject_cast<QLabel*>(table_->cellWidget(row, static_cast<int>(JOINT_NAME)));
  cell->setText(text);
}

double CustomJointsWidget::getHomePosition(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(HOME_POSITION)));
  return cell->value();
}

void CustomJointsWidget::setHomePosition(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(HOME_POSITION)));
  cell->setValue(value);
}

double CustomJointsWidget::getMinPosition(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(MIN_POSITION)));
  return cell->value();
}

void CustomJointsWidget::setMinPosition(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(MIN_POSITION)));
  cell->setValue(value);
}

double CustomJointsWidget::getMaxPosition(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(MAX_POSITION)));
  return cell->value();
}

void CustomJointsWidget::setMaxPosition(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(MAX_POSITION)));
  cell->setValue(value);
}

tobas::joint_control_type_t CustomJointsWidget::getCommandType(int row) const
{
  const auto cell = qobject_cast<qt::ComboBox*>(table_->cellWidget(row, static_cast<int>(COMMAND_TYPE)));
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

void CustomJointsWidget::setCommandType(int row, const tobas::joint_control_type_t& value)
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

  const auto cell = qobject_cast<qt::ComboBox*>(table_->cellWidget(row, static_cast<int>(COMMAND_TYPE)));
  cell->setCurrentText(text);
}

double CustomJointsWidget::getPGain(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(P_GAIN)));
  return cell->value();
}

void CustomJointsWidget::setPGain(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(P_GAIN)));
  cell->setValue(value);
}

double CustomJointsWidget::getIGain(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(I_GAIN)));
  return cell->value();
}

void CustomJointsWidget::setIGain(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(I_GAIN)));
  cell->setValue(value);
}

double CustomJointsWidget::getDGain(int row) const
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(D_GAIN)));
  return cell->value();
}

void CustomJointsWidget::setDGain(int row, double value)
{
  const auto cell = qobject_cast<qt::DoubleSpinBox*>(table_->cellWidget(row, static_cast<int>(D_GAIN)));
  cell->setValue(value);
}

QStringList CustomJointsWidget::getLinkNames() const
{
  QStringList res;
  for (int row = 0; row < count(); ++row)
    res.append(getLinkName(row));
  return res;
}

QStringList CustomJointsWidget::getJointNames() const
{
  QStringList res;
  for (int row = 0; row < count(); ++row)
    res.append(getJointName(row));
  return res;
}

QString CustomJointsWidget::getControllerType(int row) const
{
  QString group, controller;

  const auto jnt_name = getJointName(row);
  const auto hi = robot_.hardwareInterface(jnt_name.toStdString());
  switch (hi)
  {
    case hw_interface::POSITION:
      group = "position_controllers";
      break;
    case hw_interface::VELOCITY:
      group = "velocity_controllers";
      break;
    case hw_interface::EFFORT:
      group = "effort_controllers";
      break;
    default:
      throw;
  }

  const auto cmd_type = getCommandType(row);
  switch (cmd_type)
  {
    case tobas::joint_control_type_t::POSITION_CONTROL:
      controller = "JointGroupPositionController";
      break;
    case tobas::joint_control_type_t::VELOCITY_CONTROL:
      controller = "JointGroupVelocityController";
      break;
    case tobas::joint_control_type_t::EFFORT_CONTROL:
      controller = "JointGroupEffortController";
      break;
    default:
      throw;
  }

  return group + "/" + controller;
}

bool CustomJointsWidget::pidEnabled(int row) const
{
  const auto jnt_name = getJointName(row);
  const auto hi = robot_.hardwareInterface(jnt_name.toStdString());
  const auto cmd_type = getCommandType(row);

  if (hi == hw_interface::POSITION && cmd_type == tobas::joint_control_type_t::POSITION_CONTROL)
    return false;
  if (hi == hw_interface::VELOCITY && cmd_type == tobas::joint_control_type_t::VELOCITY_CONTROL)
    return false;
  if (hi == hw_interface::EFFORT && cmd_type == tobas::joint_control_type_t::EFFORT_CONTROL)
    return false;

  return true;
}

int CustomJointsWidget::getRow(const QString& jnt_name)
{
  for (int row = 0; row < count(); ++row)
    if (getJointName(row) == jnt_name)
      return row;

  return -1;
}
}  // namespace setup_assistant
}  // namespace gui
