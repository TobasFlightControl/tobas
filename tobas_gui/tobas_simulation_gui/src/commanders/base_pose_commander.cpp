#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_simulation_gui/commanders/base_pose_commander.hpp"
#include "tobas_simulation_gui/constants.hpp"

namespace gui
{
namespace sim
{
BasePoseCommanderWidget::BasePoseCommanderWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone)
{
  const auto title = new qt::Label("Base Pose", kLabelPSize, QFont::Bold);

  arming_button_ = new qt::ToggleButton("Arm", "Disarm");
  arming_button_->setFixedSize(kArmingButtonWidth, kArmingButtonHeight);

  cmd_x_ = new qt::DoubleSliderDisplay();
  cmd_y_ = new qt::DoubleSliderDisplay();
  cmd_z_ = new qt::DoubleSliderDisplay();
  cmd_roll_ = new qt::DoubleSliderDisplay();
  cmd_pitch_ = new qt::DoubleSliderDisplay();
  cmd_yaw_ = new qt::DoubleSliderDisplay();

  cmd_x_->setText("X");
  cmd_y_->setText("Y");
  cmd_z_->setText("Z");
  cmd_roll_->setText("Roll");
  cmd_pitch_->setText("Pitch");
  cmd_yaw_->setText("Yaw");

  cmd_x_->setRange(-10., 10.);
  cmd_y_->setRange(-10., 10.);
  cmd_z_->setRange(-3., 10.);
  cmd_roll_->setRange(-M_PI, M_PI);
  cmd_pitch_->setRange(-M_PI, M_PI);
  cmd_yaw_->setRange(-M_PI, M_PI);

  home_button_ = new QPushButton("Home");
  home_button_->setFixedHeight(kCommandButtonHeight);

  reset();

  // Layout
  const auto header_cols = new QHBoxLayout();
  header_cols->addWidget(title);
  header_cols->addStretch();
  header_cols->addWidget(arming_button_);

  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(home_button_);

  const auto root_rows = new QVBoxLayout();
  root_rows->addLayout(header_cols);
  root_rows->addWidget(cmd_x_);
  root_rows->addWidget(cmd_y_);
  root_rows->addWidget(cmd_z_);
  root_rows->addWidget(cmd_roll_);
  root_rows->addWidget(cmd_pitch_);
  root_rows->addWidget(cmd_yaw_);
  root_rows->addLayout(button_cols);

  setLayout(root_rows);

  // Connection
  connect(arming_button_, &qt::ToggleButton::checked, this, &self::onArmRequested);
  connect(arming_button_, &qt::ToggleButton::unchecked, this, &self::onDisarmRequested);
  connect(cmd_x_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  connect(cmd_y_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  connect(cmd_z_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  connect(cmd_roll_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  connect(cmd_pitch_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  connect(cmd_yaw_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  connect(home_button_, &QPushButton::clicked, this, &self::onHomeButtonClicked);
}

void BasePoseCommanderWidget::updateInternalDataStructures()
{
  const auto& ns = drone_.name;

  angle_pub_ = ros2::createPublisher<tobas_command_msgs::Angle>(node_, path::join(ns, tobas::kAngleCmdTopic));
  pos_vel_pub_ = ros2::createPublisher<tobas_command_msgs::PosVel>(node_, path::join(ns, tobas::kPosVelCmdTopic));
  pos_vel_yaw_pub_ =
    ros2::createPublisher<tobas_command_msgs::PosVelYaw>(node_, path::join(ns, tobas::kPosVelYawCmdTopic));

  arming_sub_ = ros2::createSubscriber(node_, path::join(ns, tobas::kArmingTopic), &self::armingCb, this);
  odom_sub_ = ros2::createSubscriber(node_, path::join(ns, tobas::kOdometryTopic), &self::odomCb, this);

  set_arm_sc_ =
    std::make_shared<ros2::SyncServiceClient<tobas_msgs::srv::SetArm>>(node_, path::join(ns, tobas::kSetArmSrv));
}

bool BasePoseCommanderWidget::start()
{
  if (!set_arm_sc_->waitForService(kWaitForService))
  {
    qt::qErrorBox(this, "Failed to connect to \"" + QString(tobas::kSetArmSrv) + "\" service server.");
    return false;
  }

  return true;
}

void BasePoseCommanderWidget::reset()
{
  arming_button_->setChecked(false);

  home_button_->setEnabled(false);

  cmd_x_->setValue(0.);
  cmd_y_->setValue(0.);
  cmd_z_->setValue(0.);
  cmd_roll_->setValue(0.);
  cmd_pitch_->setValue(0.);
  cmd_yaw_->setValue(0.);

  cmd_x_->setEnabled(false);
  cmd_y_->setEnabled(false);
  cmd_z_->setEnabled(false);
  cmd_roll_->setEnabled(false);
  cmd_pitch_->setEnabled(false);
  cmd_yaw_->setEnabled(false);
}

void BasePoseCommanderWidget::publishCurrentCommand()
{
  if (angle_pub_)
  {
    auto msg = std::make_unique<tobas_command_msgs::Angle>();
    msg->level.data = tobas_command_msgs::msg::CommandLevel::NORMAL;
    msg->angle.roll = cmd_roll_->getValue();
    msg->angle.pitch = cmd_pitch_->getValue();
    msg->angle.yaw = cmd_yaw_->getValue();
    angle_pub_->publish(std::move(msg));
  }

  if (pos_vel_pub_)
  {
    auto msg = std::make_unique<tobas_command_msgs::PosVel>();
    msg->level.data = tobas_command_msgs::msg::CommandLevel::NORMAL;
    msg->pos.x() = cmd_x_->getValue();
    msg->pos.y() = cmd_y_->getValue();
    msg->pos.z() = cmd_z_->getValue();
    msg->vel.setZero();
    pos_vel_pub_->publish(std::move(msg));
  }

  if (pos_vel_yaw_pub_)
  {
    auto msg = std::make_unique<tobas_command_msgs::PosVelYaw>();
    msg->level.data = tobas_command_msgs::msg::CommandLevel::NORMAL;
    msg->pos.x() = cmd_x_->getValue();
    msg->pos.y() = cmd_y_->getValue();
    msg->pos.z() = cmd_z_->getValue();
    msg->yaw = cmd_yaw_->getValue();
    pos_vel_yaw_pub_->publish(std::move(msg));
  }
}

bool BasePoseCommanderWidget::armRotors(bool arming)
{
  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;
  if (!set_arm_sc_->call(req, kWaitForService))
  {
    qt::qErrorBox(this, "Failed to connect to the rotor controller.");
    return false;
  }

  const auto res = set_arm_sc_->getResponse();
  if (!res->success)
  {
    qt::qErrorBox(this, "Arming service failed: " + QString::fromStdString(res->message));
    return false;
  }

  return true;
}

void BasePoseCommanderWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void BasePoseCommanderWidget::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void BasePoseCommanderWidget::onArmRequested()
{
  if (!arming_)
  {
    qt::qWarnBox(this, "Arming status is not received yet.");
    reset();
    return;
  }
  if (arming_->data)
  {
    qt::qWarnBox(this, "The rotors are already armed.");
    reset();
    return;
  }
  if (!odom_)
  {
    qt::qWarnBox(this, "Odometry is not received yet.");
    reset();
    return;
  }

  // アーム
  if (!armRotors(true))
  {
    reset();
    return;
  }

  // 初期コマンドを現在の位置姿勢に設定
  cmd_x_->setValue(odom_->frame.p.x());
  cmd_y_->setValue(odom_->frame.p.y());
  cmd_z_->setValue(odom_->frame.p.z());
  cmd_roll_->setValue(0.);
  cmd_pitch_->setValue(0.);
  cmd_yaw_->setValue(odom_->frame.M.getYaw());

  // 有効化
  home_button_->setEnabled(true);
  cmd_x_->setEnabled(true);
  cmd_y_->setEnabled(true);
  cmd_z_->setEnabled(true);
  cmd_roll_->setEnabled(true);
  cmd_pitch_->setEnabled(true);
  cmd_yaw_->setEnabled(true);

  qt::qInfoBox(this, "GUI teleoperation is ready.");
}

void BasePoseCommanderWidget::onDisarmRequested()
{
  if (!armRotors(false))
    return;

  reset();

  qt::qInfoBox(this, "GUI teleoperation is finished.");
}

void BasePoseCommanderWidget::onValueChanged()
{
  publishCurrentCommand();
}

void BasePoseCommanderWidget::onHomeButtonClicked()
{
  cmd_x_->setValue(0.);
  cmd_y_->setValue(0.);
  cmd_z_->setValue(kHomeAltitude);
  cmd_roll_->setValue(0.);
  cmd_pitch_->setValue(0.);
  cmd_yaw_->setValue(0.);
}
}  // namespace sim
}  // namespace gui
