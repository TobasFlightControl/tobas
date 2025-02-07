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
BasePoseCommanderWidget::BasePoseCommanderWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  const auto title = new qt::Label("Base Pose", kLabelPSize, QFont::Bold);

  arming_button_ = new qt::ToggleButton("Arm", "Disarm");
  home_button_ = new QPushButton("Home");

  arming_button_->setFixedSize(kButtonWidth, kButtonHeight);
  home_button_->setFixedSize(kButtonWidth, kButtonHeight);

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

  reset();

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(title);
  cols->addStretch();
  cols->addWidget(arming_button_);
  cols->addWidget(home_button_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(cols);
  rows->addWidget(cmd_x_);
  rows->addWidget(cmd_y_);
  rows->addWidget(cmd_z_);
  rows->addWidget(cmd_roll_);
  rows->addWidget(cmd_pitch_);
  rows->addWidget(cmd_yaw_);

  setLayout(rows);

  // Connection
  connect(arming_button_, &qt::ToggleButton::checked, this, &self::onArmButtonClicked);
  connect(arming_button_, &qt::ToggleButton::unchecked, this, &self::onDisarmButtonClicked);
  connect(home_button_, &QPushButton::clicked, this, &self::onHomeButtonClicked);
  connect(cmd_x_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  connect(cmd_y_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  connect(cmd_z_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  connect(cmd_roll_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  connect(cmd_pitch_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
  connect(cmd_yaw_, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
}

bool BasePoseCommanderWidget::start(const std::string& ns)
{
  pvay_pub_ =
    ros2::createPublisher<tobas_command_msgs::PosVelAccYaw>(node_, path::join(ns, tobas::kPosVelAccYawCmdTopic));
  pta_pub_ =
    ros2::createPublisher<tobas_command_msgs::PoseTwistAccel>(node_, path::join(ns, tobas::kPoseTwistAccelCmdTopic));

  arming_sub_ = ros2::createSubscriber(node_, path::join(ns, tobas::kArmingTopic), &self::armingCb, this);
  odom_sub_ = ros2::createSubscriber(node_, path::join(ns, tobas::kOdometryTopic), &self::odomCb, this);

  set_arm_sc_ =
    std::make_shared<ros2::SyncServiceClient<tobas_msgs::srv::SetArm>>(node_, path::join(ns, tobas::kSetArmSrv));
  if (!set_arm_sc_->waitForService(kWaitForService))
  {
    qt::qErrorBox(this, "Failed to connect to \"" + QString(tobas::kSetArmSrv) + "\" service server.");
    return false;
  }

  return true;
}

void BasePoseCommanderWidget::terminate()
{
  reset();

  arming_ = nullptr;
  odom_ = nullptr;

  pvay_pub_ = nullptr;
  pta_pub_ = nullptr;

  arming_sub_ = nullptr;
  odom_sub_ = nullptr;

  set_arm_sc_ = nullptr;
}

void BasePoseCommanderWidget::reset()
{
  arming_button_->setChecked(false, true);
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
  auto pvay = std::make_unique<tobas_command_msgs::PosVelAccYaw>();
  pvay->level.data = tobas_command_msgs::msg::CommandLevel::NORMAL;
  pvay->pos.x() = cmd_x_->getValue();
  pvay->pos.y() = cmd_y_->getValue();
  pvay->pos.z() = cmd_z_->getValue();
  pvay->yaw = cmd_yaw_->getValue();
  pvay_pub_->publish(std::move(pvay));

  auto pta = std::make_unique<tobas_command_msgs::PoseTwistAccel>();
  pta->level.data = tobas_command_msgs::msg::CommandLevel::NORMAL;
  pta->pos.x() = cmd_x_->getValue();
  pta->pos.y() = cmd_y_->getValue();
  pta->pos.z() = cmd_z_->getValue();
  pta->rpy.roll = cmd_roll_->getValue();
  pta->rpy.pitch = cmd_pitch_->getValue();
  pta->rpy.yaw = cmd_yaw_->getValue();
  pta_pub_->publish(std::move(pta));
}

bool BasePoseCommanderWidget::armRotors(bool arming)
{
  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;
  req->ignore_prearm_check = false;
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

void BasePoseCommanderWidget::onValueChanged()
{
  if (pvay_pub_ != nullptr && pta_pub_ != nullptr)
    publishCurrentCommand();
}

void BasePoseCommanderWidget::onArmButtonClicked()
{
  if (arming_ == nullptr)
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
  if (odom_ == nullptr)
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

void BasePoseCommanderWidget::onDisarmButtonClicked()
{
  if (!armRotors(false))
    return;

  reset();

  qt::qInfoBox(this, "GUI teleoperation is finished.");
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
