#include "tobas_simulation_gui/commanders/joint_commander.hpp"

#include <QDebug>
#include <QHBoxLayout>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_simulation_gui/commanders/constants.hpp"

namespace tobas
{
namespace gui
{
namespace sim
{
JointCommanderWidget::JointCommanderWidget(rclcpp::Node::SharedPtr node, const kdl::Tree& tree, const tobas::Drone& drone)
  : node_(node), tree_(tree), drone_(drone), rnd_gen_(rnd_dev_()), joint_parser_(tree)
{
  const auto title = new tobas::qt::Label("User Joint", cmn::kLabelPSize, QFont::Bold);

  start_stop_button_ = new tobas::qt::ToggleButton("Start", "Stop");
  start_stop_button_->setFixedSize(kHeaderButtonWidth, kHeaderButtonHeight);

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
  connect(start_stop_button_, &tobas::qt::ToggleButton::checked, this, &self::onStartRequested);
  connect(start_stop_button_, &tobas::qt::ToggleButton::unchecked, this, &self::onStopRequested);
  connect(home_button_, &QPushButton::clicked, this, &self::onHomeButtonClicked);
  connect(center_button_, &QPushButton::clicked, this, &self::onCenterButtonClicked);
  connect(random_button_, &QPushButton::clicked, this, &self::onRandomButtonClicked);
  connect(&publish_cmd_timer_, &QTimer::timeout, this, &self::onPublishCommandTimerTimeout);
}

void JointCommanderWidget::updateInternalDataStructures()
{
  if (!joint_parser_.updateInternalDataStructures()) {
    tobas::qt::qErrorBox(this, "Failed to update joint parser.");
    return;
  }

  // Clear joints for previous robot
  commanders_.clear();
  tobas::qt::clearLayout(cmd_rows_);

  tar_js_pos_.commands.clear();
  tar_js_vel_.commands.clear();
  tar_js_eff_.commands.clear();

  // Add joints of current robot
  for (const auto& [jnt_name, joint] : drone_.joints) {
    // User active joint only
    if (joint.role != tobas::JointRole::kUserActive) {
      continue;
    }

    // Check joint type is not FIXED
    const auto& jnt_type = joint_parser_.joint(jnt_name).type;
    if (jnt_type == kdl::Joint::kFixed) {
      qWarning() << "The joint type of" << QString::fromStdString(jnt_name) << "is FIXED.";
      continue;
    }

    const auto commander = new tobas::qt::DoubleSliderDisplay();
    commander->setText(QString::fromStdString(jnt_name));

    switch (joint.cmd_iface) {
      case tobas::JointCommandInterface::kPosition: {
        commander->setMinimum(joint_parser_.lowerLimit(jnt_name));
        commander->setMaximum(joint_parser_.upperLimit(jnt_name));
        switch (jnt_type) {
          case kdl::Joint::kRotation:
            commander->setSuffix(" rad");
            break;
          case kdl::Joint::kTranslation:
            commander->setSuffix(" m");
            break;
          case kdl::Joint::kFixed:
          default:
            throw std::runtime_error("Unexpected joint command interface type: " + std::to_string((int)joint.cmd_iface));
        }

        tobas_msgs::msg::JointCommand cmd;
        cmd.name = jnt_name;
        cmd.data = joint.home_pos;
        tar_js_pos_.commands.push_back(cmd);

        break;
      }
      case tobas::JointCommandInterface::kVelocity: {
        const auto max_vel = joint_parser_.maxVelocity(jnt_name);
        commander->setMinimum(-max_vel);
        commander->setMaximum(+max_vel);
        switch (jnt_type) {
          case kdl::Joint::kRotation:
            commander->setSuffix(" rad/s");
            break;
          case kdl::Joint::kTranslation:
            commander->setSuffix(" m/s");
            break;
          case kdl::Joint::kFixed:
          default:
            throw std::runtime_error("Unexpected joint command interface type: " + std::to_string((int)joint.cmd_iface));
        }

        tobas_msgs::msg::JointCommand cmd;
        cmd.name = jnt_name;
        cmd.data = 0.;
        tar_js_vel_.commands.push_back(cmd);

        break;
      }
      case tobas::JointCommandInterface::kEffort: {
        const auto max_eff = joint_parser_.maxEffort(jnt_name);
        commander->setMinimum(-max_eff);
        commander->setMaximum(+max_eff);
        switch (jnt_type) {
          case kdl::Joint::kRotation:
            commander->setSuffix(" Nm");
            break;
          case kdl::Joint::kTranslation:
            commander->setSuffix(" N");
            break;
          case kdl::Joint::kFixed:
          default:
            throw std::runtime_error("Unexpected joint command interface type: " + std::to_string((int)joint.cmd_iface));
        }

        tobas_msgs::msg::JointCommand cmd;
        cmd.name = jnt_name;
        cmd.data = 0.;
        tar_js_eff_.commands.push_back(cmd);

        break;
      }
      case tobas::JointCommandInterface::kNone: {
        break;
      }
      default: {
        tobas::qt::qErrorBox(
          this, "The command interface of joint " + QString::fromStdString(jnt_name) + " is invalid.");
        continue;
      }
    }

    commander->setValue(0.);
    commander->setEnabled(false);
    connect(
      commander,
      &tobas::qt::DoubleSliderDisplay::valueChanged,
      std::bind(&self::onValueChanged, this, std::placeholders::_1, jnt_name));
    commanders_[jnt_name] = commander;
    cmd_rows_->addWidget(commander);
  }

  // Enable joint commander only if at least one commander exists
  start_stop_button_->setEnabled(!commanders_.empty());

  // Register command publishers
  const auto& ns = drone_.name;
  if (!tar_js_pos_.commands.empty()) {
    tar_js_pos_pub_ = ros2::createPublisher<CmdMsg>(node_, path::join(ns, tobas::topic::kJointPosCmd));
  }
  if (!tar_js_vel_.commands.empty()) {
    tar_js_vel_pub_ = ros2::createPublisher<CmdMsg>(node_, path::join(ns, tobas::topic::kJointVelCmd));
  }
  if (!tar_js_eff_.commands.empty()) {
    tar_js_eff_pub_ = ros2::createPublisher<CmdMsg>(node_, path::join(ns, tobas::topic::kJointEffCmd));
  }
}

bool JointCommanderWidget::start()
{
  return true;
}

void JointCommanderWidget::reset()
{
  start_stop_button_->setChecked(false);

  for (const auto& [_, commander] : commanders_) {
    commander->setValue(0.);
    commander->setEnabled(false);
  }

  for (auto& cmd : tar_js_pos_.commands) {
    cmd.data = 0.;
  }
  for (auto& cmd : tar_js_vel_.commands) {
    cmd.data = 0.;
  }
  for (auto& cmd : tar_js_eff_.commands) {
    cmd.data = 0.;
  }

  home_button_->setEnabled(false);
  center_button_->setEnabled(false);
  random_button_->setEnabled(false);

  publish_cmd_timer_.stop();
}

void JointCommanderWidget::publishCurrentCommand()
{
  if (tar_js_pos_pub_) {
    auto tar_js_pos = std::make_unique<CmdMsg>(tar_js_pos_);
    tar_js_pos_pub_->publish(std::move(tar_js_pos));
  }

  if (tar_js_vel_pub_) {
    auto tar_js_vel = std::make_unique<CmdMsg>(tar_js_vel_);
    tar_js_vel_pub_->publish(std::move(tar_js_vel));
  }

  if (tar_js_eff_pub_) {
    auto tar_js_eff = std::make_unique<CmdMsg>(tar_js_eff_);
    tar_js_eff_pub_->publish(std::move(tar_js_eff));
  }
}

void JointCommanderWidget::onStartRequested()
{
  // 初期コマンドをホームポジションに設定して有効化
  for (const auto& [jnt_name, commander] : commanders_) {
    commander->setValue(drone_.joints.at(jnt_name).home_pos);
    commander->setEnabled(true);
  }

  // コマンドボタンを有効化
  home_button_->setEnabled(true);
  center_button_->setEnabled(true);
  random_button_->setEnabled(true);

  // 一定時間間隔でコマンド送信開始
  publish_cmd_timer_.start(kPublishCommandPeriod);

  tobas::qt::qInfoBox(this, "GUI teleoperation is ready.");
}

void JointCommanderWidget::onStopRequested()
{
  reset();

  tobas::qt::qInfoBox(this, "GUI teleoperation is ready.");
}

void JointCommanderWidget::onValueChanged(double value, const std::string& jnt_name)
{
  if (!drone_.joints.contains(jnt_name)) {
    qWarning() << "Invalid joint name:" << QString::fromStdString(jnt_name);
    return;
  }

  const auto& joint = drone_.joints.at(jnt_name);
  bool jnt_found = false;

  switch (joint.cmd_iface) {
    case tobas::JointCommandInterface::kPosition: {
      for (auto& cmd : tar_js_pos_.commands) {
        if (cmd.name == jnt_name) {
          cmd.data = value;
          jnt_found = true;
          break;
        }
      }

      if (!jnt_found) {
        qWarning() << "Position commanded joint" << QString::fromStdString(jnt_name) << "is not found.";
        return;
      }

      break;
    }
    case tobas::JointCommandInterface::kVelocity: {
      for (auto& cmd : tar_js_vel_.commands) {
        if (cmd.name == jnt_name) {
          cmd.data = value;
          jnt_found = true;
          break;
        }
      }

      if (!jnt_found) {
        qWarning() << "Velocity commanded joint" << QString::fromStdString(jnt_name) << "is not found.";
        return;
      }

      break;
    }
    case tobas::JointCommandInterface::kEffort: {
      for (auto& cmd : tar_js_eff_.commands) {
        if (cmd.name == jnt_name) {
          cmd.data = value;
          jnt_found = true;
          break;
        }
      }

      if (!jnt_found) {
        qWarning() << "Effort commanded joint" << QString::fromStdString(jnt_name) << "is not found.";
        return;
      }

      break;
    }
    case tobas::JointCommandInterface::kNone: {
      break;
    }
    default: {
      qWarning() << "The command interface of joint" << QString::fromStdString(jnt_name) << "is invalid.";
      return;
    }
  }

  publishCurrentCommand();
}

void JointCommanderWidget::onHomeButtonClicked()
{
  for (const auto& [jnt_name, commander] : commanders_) {
    commander->setValue(drone_.joints.at(jnt_name).home_pos);
  }

  publishCurrentCommand();
}

void JointCommanderWidget::onCenterButtonClicked()
{
  for (const auto& [_, commander] : commanders_) {
    commander->setValue((commander->getMinimum() + commander->getMaximum()) / 2);
  }

  publishCurrentCommand();
}

void JointCommanderWidget::onRandomButtonClicked()
{
  for (const auto& [_, commander] : commanders_) {
    std::uniform_real_distribution<double> uniform(commander->getMinimum(), commander->getMaximum());
    const auto value = uniform(rnd_gen_);
    commander->setValue(value);
  }
}

void JointCommanderWidget::onPublishCommandTimerTimeout()
{
  publishCurrentCommand();
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
