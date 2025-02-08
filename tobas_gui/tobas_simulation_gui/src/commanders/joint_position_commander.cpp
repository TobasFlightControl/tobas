#include <QHBoxLayout>
#include <QDebug>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_simulation_gui/commanders/joint_position_commander.hpp"
#include "tobas_simulation_gui/constants.hpp"

namespace gui
{
namespace sim
{
JointPositionCommanderWidget::JointPositionCommanderWidget(
  rclcpp::Node::SharedPtr node,
  const kdl::Tree& tree,
  const tobas::Drone& drone)
  : node_(node), tree_(tree), drone_(drone), joint_parser_(tree)
{
  const auto title = new qt::Label("Joint Position", kLabelPSize, QFont::Bold);

  start_stop_button_ = new qt::ToggleButton("Start", "Stop");
  start_stop_button_->setFixedSize(kStartStopButtonWidth, kStartStopButtonHeight);

  cmd_rows_ = new QVBoxLayout();

  home_button_ = new QPushButton("Home");
  center_button_ = new QPushButton("Center");
  random_button_ = new QPushButton("Random");
  home_button_->setFixedHeight(kCommandButtonHeight);
  center_button_->setFixedHeight(kCommandButtonHeight);
  random_button_->setFixedHeight(kCommandButtonHeight);

  reset();

  // Layout
  const auto header_cols = new QHBoxLayout();
  header_cols->addWidget(title);
  header_cols->addStretch();
  header_cols->addWidget(start_stop_button_);

  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(home_button_);
  button_cols->addWidget(center_button_);
  button_cols->addWidget(random_button_);

  const auto root_rows = new QVBoxLayout();
  root_rows->addLayout(header_cols);
  root_rows->addLayout(cmd_rows_);
  root_rows->addLayout(button_cols);

  setLayout(root_rows);

  // Connection
  connect(start_stop_button_, &qt::ToggleButton::checked, this, &self::onStartRequested);
  connect(start_stop_button_, &qt::ToggleButton::unchecked, this, &self::onStopRequested);
  connect(home_button_, &QPushButton::clicked, this, &self::onHomeButtonClicked);
  connect(center_button_, &QPushButton::clicked, this, &self::onCenterButtonClicked);
  connect(random_button_, &QPushButton::clicked, this, &self::onRandomButtonClicked);
  connect(&publish_cmd_timer_, &QTimer::timeout, this, &self::onPublishCommandTimerTimeout);
}

bool JointPositionCommanderWidget::start()
{
  joint_parser_.updateInternalDataStructures();

  for (const auto& [jnt_name, joint] : drone_.joints)
  {
    // MANIPULATION用の関節のみ登録
    if (joint.role != tobas::jnt_role_t::MANIPULATION)
      continue;

    tobas_msgs::msg::JointState cmd;
    cmd.name = jnt_name;
    switch (joint.cmd_iface)
    {
      case tobas::jnt_cmd_iface_t::POSITION:
        cmd.position = joint.home_pos;
        cmd.velocity = NAN;
        cmd.effort = NAN;
        tar_js_pos_.states.push_back(cmd);
        break;
      case tobas::jnt_cmd_iface_t::VELOCITY:
        cmd.position = joint.home_pos;
        cmd.velocity = NAN;
        cmd.effort = NAN;
        tar_js_vel_.states.push_back(cmd);
        break;
      case tobas::jnt_cmd_iface_t::EFFORT:
        cmd.position = joint.home_pos;
        cmd.velocity = 0.;
        cmd.effort = NAN;
        tar_js_eff_.states.push_back(cmd);
        break;
      default:
        qt::qErrorBox(this, "The command interface of joint " + QString::fromStdString(jnt_name) + " is invalid.");
        continue;
    }

    const auto commander = new qt::DoubleSliderDisplay();
    commander->setText(QString::fromStdString(jnt_name));
    commander->setMinimum(joint_parser_.lowerLimit(jnt_name));
    commander->setMaximum(joint_parser_.upperLimit(jnt_name));
    commander->setValue(0.);
    commander->setEnabled(false);
    connect(
      commander, &qt::DoubleSliderDisplay::valueChanged,
      std::bind(&self::onValueChanged, this, std::placeholders::_1, jnt_name));
    commanders_[jnt_name] = commander;
    cmd_rows_->addWidget(commander);
  }

  const auto& ns = drone_.name;
  if (tar_js_pos_.states.size() > 0)
    tar_js_pos_pub_ =
      ros2::createPublisher<tobas_msgs::msg::JointStateArray>(node_, path::join(ns, tobas::kPosCtrlJSTopic));
  if (tar_js_vel_.states.size() > 0)
    tar_js_vel_pub_ =
      ros2::createPublisher<tobas_msgs::msg::JointStateArray>(node_, path::join(ns, tobas::kVelCtrlJSTopic));
  if (tar_js_eff_.states.size() > 0)
    tar_js_eff_pub_ =
      ros2::createPublisher<tobas_msgs::msg::JointStateArray>(node_, path::join(ns, tobas::kEffCtrlJSTopic));

  return true;
}

void JointPositionCommanderWidget::terminate()
{
  reset();

  commanders_.clear();
  qt::clearLayout(cmd_rows_);

  tar_js_pos_.states.clear();
  tar_js_vel_.states.clear();
  tar_js_eff_.states.clear();

  tar_js_pos_pub_ = nullptr;
  tar_js_vel_pub_ = nullptr;
  tar_js_eff_pub_ = nullptr;
}

void JointPositionCommanderWidget::reset()
{
  start_stop_button_->setChecked(false, true);

  for (const auto& [_, commander] : commanders_)
  {
    commander->setValue(0.);
    commander->setEnabled(false);
  }

  for (auto& state : tar_js_pos_.states)
    state.position = 0.;
  for (auto& state : tar_js_vel_.states)
    state.position = 0.;
  for (auto& state : tar_js_eff_.states)
    state.position = 0.;

  home_button_->setEnabled(false);
  center_button_->setEnabled(false);
  random_button_->setEnabled(false);

  publish_cmd_timer_.stop();
}

void JointPositionCommanderWidget::publishCurrentCommand()
{
  if (tar_js_pos_pub_ != nullptr)
  {
    auto tar_js_pos = std::make_unique<tobas_msgs::msg::JointStateArray>(tar_js_pos_);
    tar_js_pos_pub_->publish(std::move(tar_js_pos));
  }

  if (tar_js_vel_pub_ != nullptr)
  {
    auto tar_js_vel = std::make_unique<tobas_msgs::msg::JointStateArray>(tar_js_vel_);
    tar_js_vel_pub_->publish(std::move(tar_js_vel));
  }

  if (tar_js_eff_pub_ != nullptr)
  {
    auto tar_js_eff = std::make_unique<tobas_msgs::msg::JointStateArray>(tar_js_eff_);
    tar_js_eff_pub_->publish(std::move(tar_js_eff));
  }
}

void JointPositionCommanderWidget::onStartRequested()
{
  // 初期コマンドをホームポジションに設定して有効化
  for (const auto& [jnt_name, commander] : commanders_)
  {
    commander->setValue(drone_.joints.at(jnt_name).home_pos);
    commander->setEnabled(true);
  }

  // コマンドボタンを有効化
  home_button_->setEnabled(true);
  center_button_->setEnabled(true);
  random_button_->setEnabled(true);

  // 一定時間間隔でコマンド送信開始
  publish_cmd_timer_.start(kPublishCommandPeriod);

  qt::qInfoBox(this, "GUI teleoperation is ready.");
}

void JointPositionCommanderWidget::onStopRequested()
{
  reset();

  qt::qInfoBox(this, "GUI teleoperation is ready.");
}

void JointPositionCommanderWidget::onValueChanged(double value, const std::string& jnt_name)
{
  if (!drone_.joints.contains(jnt_name))
  {
    qWarning() << "Invalid joint name: " << QString::fromStdString(jnt_name);
    return;
  }

  const auto& joint = drone_.joints.at(jnt_name);

  switch (joint.cmd_iface)
  {
    case tobas::jnt_cmd_iface_t::POSITION:
    {
      for (auto& cmd : tar_js_pos_.states)
      {
        if (cmd.name == jnt_name)
        {
          cmd.position = value;
          break;
        }
      }

      qWarning() << "Position commanded joint " << QString::fromStdString(jnt_name) << " is not found.";
      return;
    }
    case tobas::jnt_cmd_iface_t::VELOCITY:
    {
      for (auto& cmd : tar_js_vel_.states)
      {
        if (cmd.name == jnt_name)
        {
          cmd.position = value;
          break;
        }
      }

      qWarning() << "Velocity commanded joint " << QString::fromStdString(jnt_name) << " is not found.";
      return;
    }
    case tobas::jnt_cmd_iface_t::EFFORT:
    {
      for (auto& cmd : tar_js_eff_.states)
      {
        if (cmd.name == jnt_name)
        {
          cmd.position = value;
          break;
        }
      }

      qWarning() << "Effort commanded joint " << QString::fromStdString(jnt_name) << " is not found.";
      return;
    }
    default:
    {
      qWarning() << "The command interface of joint " << QString::fromStdString(jnt_name) << " is invalid.";
      return;
    }
  }

  publishCurrentCommand();
}

void JointPositionCommanderWidget::onHomeButtonClicked()
{
  // TODO
}

void JointPositionCommanderWidget::onCenterButtonClicked()
{
  // TODO
}

void JointPositionCommanderWidget::onRandomButtonClicked()
{
  // TODO
}

void JointPositionCommanderWidget::onPublishCommandTimerTimeout()
{
  publishCurrentCommand();
}
}  // namespace sim
}  // namespace gui
