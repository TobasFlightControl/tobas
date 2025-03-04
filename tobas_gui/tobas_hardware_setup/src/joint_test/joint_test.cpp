#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_hardware_setup/joint_test/joint_test.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hw
{
JointTestWidget::JointTestWidget(rclcpp::Node::SharedPtr node, const kdl::Tree& tree, const tobas::Drone& drone)
  : node_(node), tree_(tree), drone_(drone)
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Press \"Start\" button to start joint test.\n\n"
    "2. For each channel, confirm that the position, velocity, or effort is correctly following the command.\n\n"
    "3. If any joint does not behave as expected, please review the URDF or Setup Assistant settings.\n\n"
    "4. Press \"Stop\" button to stop joint test.\n\n",
    kBodyPSize);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  start_button_->setEnabled(true);

  stop_button_ = new QPushButton("Stop");
  stop_button_->setFixedSize(kButtonWidth, kButtonHeight);
  stop_button_->setEnabled(false);

  commands_publisher_ = new JointCommandsPublisherWidget(node, tree, drone);

  setEnabled(false);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(start_button_);
  cols->addWidget(stop_button_);
  cols->addStretch();

  rows_->addWidget(instruction);
  rows_->addLayout(cols);
  rows_->addWidget(commands_publisher_);
  rows_->addStretch();

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(stop_button_, &QPushButton::clicked, this, &self::onStopButtonClicked);
}

const char* JointTestWidget::name() const
{
  return "Joint Test";
}

const char* JointTestWidget::title() const
{
  return "Test Joints with PWM Interface";
}

void JointTestWidget::reset()
{
  commands_publisher_->stop();

  start_button_->setEnabled(true);
  stop_button_->setEnabled(false);

  arming_ = nullptr;
}

void JointTestWidget::updateInternalDataStructures()
{
  reset();

  commands_publisher_->updateInternalDataStructures();

  arming_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);

  setEnabled(true);
}

void JointTestWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void JointTestWidget::onStartButtonClicked()
{
  // アームされていないことを確認
  if (arming_ == nullptr)
  {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status is not received yet.");
    return;
  }
  if (arming_->data)
  {
    qt::qWarnBox(this, "This operation cannot be performed while rotors are armed.");
    return;
  }

  commands_publisher_->start();

  start_button_->setEnabled(false);
  stop_button_->setEnabled(true);

  qt::qInfoBox(this, "Joint test is started.");
}

void JointTestWidget::onStopButtonClicked()
{
  reset();

  qt::qInfoBox(this, "Joint test is finished.");
}
}  // namespace hw
}  // namespace gui
