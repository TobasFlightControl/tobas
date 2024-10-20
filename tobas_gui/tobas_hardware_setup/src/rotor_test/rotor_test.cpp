#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

#include "tobas_hardware_setup/rotor_test/rotor_test.hpp"
#include "tobas_hardware_setup/constants.hpp"

using namespace std;

namespace gui
{
namespace hardware_setup
{
RotorTestWidget::RotorTestWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone), spinner_(Qt::WindowModal, this), arm_thread_(node, true), disarm_thread_(node, false)
{
  const auto warning =
    new qt::DescriptionWidget("Warning: Ensure that propellers are removed from motors.\n\n", kBodyPSize);
  warning->setStyleSheet("color: red; font-weight: bold;");

  const auto instruction = new qt::DescriptionWidget(
    "1. Connect the ESCs to the Navio2 in the correct order.\n\n"
    "2. Press \"Start\" button.\n\n"
    "3. For all motors, confirm the followings:\n"
    "   - The motor rotates in the correct direction. If not, swap any two of the three ESC-motor connections.\n"
    "   - The motor does not rotate when the command RPM is 0.\n"
    "   - The sound of rotation gradually increases as the command RPM approaches its maximum value.\n"
    "   - Two motors of the same model produce roughly the same sound level at the same command RPM.\n\n"
    "4. Press \"Stop\" button.\n\n",
    kBodyPSize);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  start_button_->setEnabled(true);

  stop_button_ = new QPushButton("Stop");
  stop_button_->setFixedSize(kButtonWidth, kButtonHeight);
  stop_button_->setEnabled(false);

  speeds_publisher_ = new RotorSpeedsPublisherWidget(node_, drone_);

  setEnabled(false);

  // Layout
  rows_->addWidget(warning);
  rows_->addWidget(instruction);
  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);
  cols->addWidget(start_button_);
  cols->addWidget(stop_button_);
  cols->addStretch();
  rows_->addWidget(speeds_publisher_);
  rows_->addStretch();

  // Connections
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(&arm_thread_, &SetArmThread::finished, this, &self::onArmFinished);
  connect(stop_button_, &QPushButton::clicked, this, &self::onStopButtonClicked);
  connect(&disarm_thread_, &SetArmThread::finished, this, &self::onDisarmFinished);
}

const char* RotorTestWidget::name() const
{
  return "Rotor Test";
}

const char* RotorTestWidget::title() const
{
  return "Test Rotors";
}

void RotorTestWidget::updateInternalDataStructures()
{
  reset();
  speeds_publisher_->updateInternalDataStructures();
  arming_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);
  setEnabled(true);
}

void RotorTestWidget::reset()
{
  speeds_publisher_->stop();

  start_button_->setEnabled(true);
  stop_button_->setEnabled(false);

  is_running_ = false;
}

void RotorTestWidget::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void RotorTestWidget::onStartButtonClicked()
{
  // アームされていないことを確認
  if (arming_ == nullptr)
  {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status is not received yet.");
    return;
  }
  if (arming_->data)
  {
    qt::qWarnBox(this, "This operation cannot be performed because the rotors are already armed.");
    return;
  }

  spinner_.show();
  spinner_.start();

  arm_thread_.setNamespace(drone_.name);
  arm_thread_.start();
}

void RotorTestWidget::onArmFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (!success)
  {
    qt::qErrorBox(this, message);
    return;
  }

  speeds_publisher_->start();

  start_button_->setEnabled(false);
  stop_button_->setEnabled(true);

  is_running_ = true;

  qt::qInfoBox(this, "Rotor test is started.");
}

void RotorTestWidget::onStopButtonClicked()
{
  spinner_.show();
  spinner_.start();

  disarm_thread_.setNamespace(drone_.name);
  disarm_thread_.start();
}

void RotorTestWidget::onDisarmFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  reset();

  if (success)
    qt::qInfoBox(this, "Rotor test is finished.");
  else
    qt::qErrorBox(this, message);
}
}  // namespace hardware_setup
}  // namespace gui
