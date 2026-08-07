// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/commanders/base_pose_commander.hpp"

#include <QDebug>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_simulation_gui/commanders/constants.hpp"

namespace ch = std::chrono;

namespace tobas
{
namespace gui
{
namespace sim
{
BasePoseCommanderWidget::BasePoseCommanderWidget(
  rclcpp::Node::SharedPtr node,
  const RosQtBridge& bridge,
  const Drone& drone)
  : node_(node), drone_(drone)
{
  const auto root_rows = new QVBoxLayout();
  setLayout(root_rows);

  const auto header_cols = new QHBoxLayout();
  root_rows->addLayout(header_cols);

  const auto title = new qt::Label("Base Pose", cmn::kLabelPSize, QFont::Bold);
  header_cols->addWidget(title);
  header_cols->addStretch();

  arming_button_ = new qt::ToggleButton("Arm", "Disarm");
  arming_button_->setFixedSize(kHeaderButtonWidth, kHeaderButtonHeight);
  header_cols->addWidget(arming_button_);
  connect(arming_button_, &qt::ToggleButton::checked, this, &self::onArmRequested);
  connect(arming_button_, &qt::ToggleButton::unchecked, this, &self::onDisarmRequested);

  constexpr std::array kLabelsXYZ = { "X", "Y", "Z" };
  for (size_t i = 0; i < 3; ++i) {
    cmd_xyz_[i] = new qt::DoubleSliderDisplay();
    cmd_xyz_[i]->setRange(-10.0, 10.0);
    cmd_xyz_[i]->setText(kLabelsXYZ[i]);
    cmd_xyz_[i]->setSuffix(" m");
    cmd_xyz_[i]->setDecimals(2);

    root_rows->addWidget(cmd_xyz_[i]);
    connect(cmd_xyz_[i], &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  }

  constexpr std::array kLabelsRPY = { "Roll", "Pitch", "Yaw" };
  for (size_t i = 0; i < 3; ++i) {
    cmd_rpy_[i] = new qt::IntSliderDisplay();
    cmd_rpy_[i]->setRange(-180, 180);
    cmd_rpy_[i]->setText(kLabelsRPY[i]);
    cmd_rpy_[i]->setSuffix(" deg");

    root_rows->addWidget(cmd_rpy_[i]);
    connect(cmd_rpy_[i], &qt::IntSliderDisplay::valueChanged, this, &self::onValueChanged);
  }

  const auto button_cols = new QHBoxLayout();
  root_rows->addLayout(button_cols);

  home_button_ = new QPushButton("Home");
  home_button_->setFixedHeight(kCommandButtonHeight);
  button_cols->addWidget(home_button_);
  connect(home_button_, &QPushButton::clicked, this, &self::onHomeButtonClicked);

  reset();

  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::odomReceived, this, &self::odomCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::rcInputReceived, this, &self::rcInputCb, Qt::QueuedConnection);
}

void BasePoseCommanderWidget::updateInternalDataStructures()
{
  const auto ns = '/' + drone_.name;

  angle_pub_ = ros2::createPublisher<tobas_command_msgs::Angle>(node_, path::join(ns, topic::kAngleCmd));
  pva_pub_ = ros2::createPublisher<tobas_command_msgs::PosVelAcc>(node_, path::join(ns, topic::kPosVelAccCmd));
  pvay_pub_ = ros2::createPublisher<tobas_command_msgs::PosVelAccYaw>(node_, path::join(ns, topic::kPosVelAccYawCmd));
  pvapy_pub_ =
    ros2::createPublisher<tobas_command_msgs::PosVelAccPitchYaw>(node_, path::join(ns, topic::kPosVelAccPitchYawCmd));

  set_arm_sc_ =
    std::make_shared<ros2::SyncServiceClient<tobas_msgs::srv::SetArm>>(node_, path::join(ns, service::kSetArm));
}

bool BasePoseCommanderWidget::start(ch::milliseconds timeout)
{
  bool success = true;
  QString message;

  qt::startThreadAndWait(
    [&]()
    {
      if (!set_arm_sc_->waitForService(timeout)) {
        success = false;
        message = "Failed to connect to \"" + QString(service::kSetArm) + "\" service server.";
        return;
      }
    });

  if (!success) {
    qWarning().noquote() << message;
    return false;
  }

  return true;
}

void BasePoseCommanderWidget::reset()
{
  arming_.reset();
  odom_.reset();
  rcin_.reset();

  arming_button_->setChecked(false);
  home_button_->setEnabled(false);

  for (const auto& cmd : cmd_xyz_) {
    cmd->setValue(0.0);
    cmd->setEnabled(false);
  }
  for (const auto& cmd : cmd_rpy_) {
    cmd->setValue(0);
    cmd->setEnabled(false);
  }
}

bool BasePoseCommanderWidget::isRunning() const
{
  return arming_button_->isChecked();
}

void BasePoseCommanderWidget::publishCurrentCommand()
{
  const auto tar_x = cmd_xyz_[0]->getValue();
  const auto tar_y = cmd_xyz_[1]->getValue();
  const auto tar_z = cmd_xyz_[2]->getValue();
  const auto tar_roll = st::deg2rad(cmd_rpy_[0]->getValue());
  const auto tar_pitch = st::deg2rad(cmd_rpy_[1]->getValue());
  const auto tar_yaw = st::deg2rad(cmd_rpy_[2]->getValue());

  if (angle_pub_) {
    auto msg = std::make_unique<tobas_command_msgs::Angle>();
    msg->priority.data = tobas_command_msgs::msg::Priority::NORMAL;
    msg->angle.roll = tar_roll;
    msg->angle.pitch = tar_pitch;
    msg->angle.yaw = tar_yaw;
    angle_pub_->publish(std::move(msg));
  }

  if (pva_pub_) {
    auto msg = std::make_unique<tobas_command_msgs::PosVelAcc>();
    msg->priority.data = tobas_command_msgs::msg::Priority::NORMAL;
    msg->pos.x() = tar_x;
    msg->pos.y() = tar_y;
    msg->pos.z() = tar_z;
    msg->vel.setZero();
    msg->acc.setZero();
    pva_pub_->publish(std::move(msg));
  }

  if (pvay_pub_) {
    auto msg = std::make_unique<tobas_command_msgs::PosVelAccYaw>();
    msg->priority.data = tobas_command_msgs::msg::Priority::NORMAL;
    msg->pos.x() = tar_x;
    msg->pos.y() = tar_y;
    msg->pos.z() = tar_z;
    msg->vel.setZero();
    msg->acc.setZero();
    msg->yaw = tar_yaw;
    pvay_pub_->publish(std::move(msg));
  }

  if (pvapy_pub_) {
    auto msg = std::make_unique<tobas_command_msgs::PosVelAccPitchYaw>();
    msg->priority.data = tobas_command_msgs::msg::Priority::NORMAL;
    msg->pos.x() = tar_x;
    msg->pos.y() = tar_y;
    msg->pos.z() = tar_z;
    msg->vel.setZero();
    msg->acc.setZero();
    msg->pitch = tar_pitch;
    msg->yaw = tar_yaw;
    pvapy_pub_->publish(std::move(msg));
  }
}

bool BasePoseCommanderWidget::armRotors(bool arming)
{
  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;

  const auto res = set_arm_sc_->sendRequestAndWait(req);
  if (!res) {
    qt::qErrorBox(this, "Failed to connect to the rotor controller.");
    return false;
  }

  if (!res->success) {
    qt::qErrorBox(this, "Arming service failed: " + QString::fromStdString(res->message));
    return false;
  }

  return true;
}

void BasePoseCommanderWidget::onArmRequested()
{
  if (!arming_) {
    qt::qWarnBox(this, "Arming status has not been received yet.");
    return;
  }
  if (!odom_) {
    qt::qWarnBox(this, "Odometry has not been received yet.");
    return;
  }
  if (!rcin_) {
    qt::qWarnBox(this, "RC input has not been received yet.");
    return;
  }

  if (rcin_->enable) {
    qt::qWarnBox(this, "GUI teleoperation cannot be started because manual control is enabled.");
    return;
  }

  // Arm.
  if (!arming_->data) {
    if (!armRotors(true)) {
      return;
    }
  }

  // Get the current pose.
  const auto& cur_pos = odom_->odom.odom.frame.p;
  const kdl::Euler cur_rpy(odom_->odom.odom.frame.M);

  // Set the initial command to the current pose.
  cmd_xyz_[0]->setValue(cur_pos.x());
  cmd_xyz_[1]->setValue(cur_pos.y());
  cmd_xyz_[2]->setValue(cur_pos.z());
  cmd_rpy_[0]->setValue(st::rad2deg(cur_rpy.roll));
  cmd_rpy_[1]->setValue(st::rad2deg(cur_rpy.pitch));
  cmd_rpy_[2]->setValue(st::rad2deg(cur_rpy.yaw));

  // Enable command controls.
  home_button_->setEnabled(true);
  for (const auto& cmd : cmd_xyz_) {
    cmd->setEnabled(true);
  }
  for (const auto& cmd : cmd_rpy_) {
    cmd->setEnabled(true);
  }

  qt::qInfoBox(this, "GUI teleoperation is ready.");
}

void BasePoseCommanderWidget::onDisarmRequested()
{
  if (!arming_) {
    qt::qWarnBox(this, "Arming status has not been received yet.");
    return;
  }

  if (arming_->data) {
    if (!armRotors(false)) {
      return;
    }
  }

  reset();

  qt::qInfoBox(this, "GUI teleoperation was finished.");
}

void BasePoseCommanderWidget::onValueChanged()
{
  publishCurrentCommand();
}

void BasePoseCommanderWidget::onHomeButtonClicked()
{
  cmd_xyz_[0]->setValue(0.0);
  cmd_xyz_[1]->setValue(0.0);
  cmd_xyz_[2]->setValue(kHomeAltitude);
  cmd_rpy_[0]->setValue(0);
  cmd_rpy_[1]->setValue(0);
  cmd_rpy_[2]->setValue(0);
}

void BasePoseCommanderWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  // Stop commanding if the vehicle is disarmed externally.
  if (isRunning() && !arming->data) {
    reset();
  }

  arming_ = arming;
}

void BasePoseCommanderWidget::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void BasePoseCommanderWidget::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  // Stop commanding if manual control is enabled.
  if (isRunning() && rcin->status == tobas_msgs::msg::RCInput::STATUS_OK && rcin->enable) {
    reset();
  }

  rcin_ = rcin;
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
