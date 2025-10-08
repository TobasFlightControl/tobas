#include "tobas_simulation_gui/commanders/base_pose_commander.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_simulation_gui/commanders/constants.hpp"

namespace gui
{
namespace sim
{
BasePoseCommanderWidget::BasePoseCommanderWidget(
  rclcpp::Node::SharedPtr node,
  const RosQtBridge& bridge,
  const tobas::Drone& drone)
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
  arming_button_->setFixedSize(kStartButtonWidth, kStartButtonHeight);
  header_cols->addWidget(arming_button_);
  connect(arming_button_, &qt::ToggleButton::checked, this, &self::onArmRequested);
  connect(arming_button_, &qt::ToggleButton::unchecked, this, &self::onDisarmRequested);

  static constexpr std::array<const char*, 3> kLabelsXYZ = { "X", "Y", "Z" };
  for (size_t i = 0; i < 3; ++i) {
    cmd_xyz_[i] = new qt::DoubleSliderDisplay();
    cmd_xyz_[i]->setRange(-10., 10.);
    cmd_xyz_[i]->setText(kLabelsXYZ[i]);
    cmd_xyz_[i]->setSuffix(" m");
    cmd_xyz_[i]->setDecimals(2);

    root_rows->addWidget(cmd_xyz_[i]);
    connect(cmd_xyz_[i], &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  }

  static constexpr std::array<const char*, 3> kLabelsRPY = { "Roll", "Pitch", "Yaw" };
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
}

void BasePoseCommanderWidget::updateInternalDataStructures()
{
  const auto& ns = drone_.name;

  angle_pub_ = ros2::createPublisher<tobas_command_msgs::Angle>(node_, path::join(ns, tobas::kAngleCmdTopic));
  pos_vel_pub_ = ros2::createPublisher<tobas_command_msgs::PosVel>(node_, path::join(ns, tobas::kPosVelCmdTopic));
  pos_vel_yaw_pub_ =
    ros2::createPublisher<tobas_command_msgs::PosVelYaw>(node_, path::join(ns, tobas::kPosVelYawCmdTopic));
  pos_vel_pitch_yaw_pub_ =
    ros2::createPublisher<tobas_command_msgs::PosVelPitchYaw>(node_, path::join(ns, tobas::kPosVelPitchYawCmdTopic));

  set_arm_sc_ =
    std::make_shared<ros2::SyncServiceClient<tobas_msgs::srv::SetArm>>(node_, path::join(ns, tobas::kSetArmSrv));
}

bool BasePoseCommanderWidget::start()
{
  if (!set_arm_sc_->waitForService()) {
    qt::qErrorBox(this, "Failed to connect to \"" + QString(tobas::kSetArmSrv) + "\" service server.");
    return false;
  }

  return true;
}

void BasePoseCommanderWidget::reset()
{
  arming_button_->setChecked(false);

  home_button_->setEnabled(false);

  for (const auto& cmd : cmd_xyz_) {
    cmd->setValue(0.);
    cmd->setEnabled(false);
  }
  for (const auto& cmd : cmd_rpy_) {
    cmd->setValue(0);
    cmd->setEnabled(false);
  }
}

void BasePoseCommanderWidget::publishCurrentCommand()
{
  const auto tar_x = cmd_xyz_[0]->getValue();
  const auto tar_y = cmd_xyz_[1]->getValue();
  const auto tar_z = cmd_xyz_[2]->getValue();
  const auto tar_roll = tobas_std::deg2rad(cmd_rpy_[0]->getValue());
  const auto tar_pitch = tobas_std::deg2rad(cmd_rpy_[1]->getValue());
  const auto tar_yaw = tobas_std::deg2rad(cmd_rpy_[2]->getValue());

  if (angle_pub_) {
    auto msg = std::make_unique<tobas_command_msgs::Angle>();
    msg->level.data = tobas_command_msgs::msg::CommandLevel::NORMAL;
    msg->angle.roll = tar_roll;
    msg->angle.pitch = tar_pitch;
    msg->angle.yaw = tar_yaw;
    angle_pub_->publish(std::move(msg));
  }

  if (pos_vel_pub_) {
    auto msg = std::make_unique<tobas_command_msgs::PosVel>();
    msg->level.data = tobas_command_msgs::msg::CommandLevel::NORMAL;
    msg->pos.x() = tar_x;
    msg->pos.y() = tar_y;
    msg->pos.z() = tar_z;
    msg->vel.setZero();
    pos_vel_pub_->publish(std::move(msg));
  }

  if (pos_vel_yaw_pub_) {
    auto msg = std::make_unique<tobas_command_msgs::PosVelYaw>();
    msg->level.data = tobas_command_msgs::msg::CommandLevel::NORMAL;
    msg->pos.x() = tar_x;
    msg->pos.y() = tar_y;
    msg->pos.z() = tar_z;
    msg->yaw = tar_yaw;
    pos_vel_yaw_pub_->publish(std::move(msg));
  }

  if (pos_vel_pitch_yaw_pub_) {
    auto msg = std::make_unique<tobas_command_msgs::PosVelPitchYaw>();
    msg->level.data = tobas_command_msgs::msg::CommandLevel::NORMAL;
    msg->pos.x() = tar_x;
    msg->pos.y() = tar_y;
    msg->pos.z() = tar_z;
    msg->pitch = tar_pitch;
    msg->yaw = tar_yaw;
    pos_vel_pitch_yaw_pub_->publish(std::move(msg));
  }
}

bool BasePoseCommanderWidget::armRotors(bool arming)
{
  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;
  if (!set_arm_sc_->call(req, kServiceCallTimeout)) {
    qt::qErrorBox(this, "Failed to connect to the rotor controller.");
    return false;
  }

  const auto res = set_arm_sc_->getResponse();
  if (!res->success) {
    qt::qErrorBox(this, "Arming service failed: " + QString::fromStdString(res->message));
    return false;
  }

  return true;
}

void BasePoseCommanderWidget::onArmRequested()
{
  if (!arming_) {
    qt::qWarnBox(this, "Arming status is not received yet.");
    reset();
    return;
  }
  if (arming_->data) {
    qt::qWarnBox(this, "The rotors are already armed.");
    reset();
    return;
  }
  if (!odom_) {
    qt::qWarnBox(this, "Odometry is not received yet.");
    reset();
    return;
  }

  // アーム
  if (!armRotors(true)) {
    reset();
    return;
  }

  // 現在の位置姿勢を取得
  const auto& cur_pos = odom_->frame.p;
  const kdl::Euler cur_rpy(odom_->frame.M);

  // 初期コマンドを現在の位置姿勢に設定
  cmd_xyz_[0]->setValue(cur_pos.x());
  cmd_xyz_[1]->setValue(cur_pos.y());
  cmd_xyz_[2]->setValue(cur_pos.z());
  cmd_rpy_[0]->setValue(tobas_std::rad2deg(cur_rpy.roll));
  cmd_rpy_[1]->setValue(tobas_std::rad2deg(cur_rpy.pitch));
  cmd_rpy_[2]->setValue(tobas_std::rad2deg(cur_rpy.yaw));

  // 有効化
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
  if (!armRotors(false)) {
    return;
  }

  reset();

  qt::qInfoBox(this, "GUI teleoperation is finished.");
}

void BasePoseCommanderWidget::onValueChanged()
{
  publishCurrentCommand();
}

void BasePoseCommanderWidget::onHomeButtonClicked()
{
  cmd_xyz_[0]->setValue(0.);
  cmd_xyz_[1]->setValue(0.);
  cmd_xyz_[2]->setValue(kHomeAltitude);
  cmd_rpy_[0]->setValue(0);
  cmd_rpy_[1]->setValue(0);
  cmd_rpy_[2]->setValue(0);
}

void BasePoseCommanderWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void BasePoseCommanderWidget::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
}
}  // namespace sim
}  // namespace gui
