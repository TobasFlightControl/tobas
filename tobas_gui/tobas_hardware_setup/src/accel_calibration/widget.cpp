#include "tobas_hardware_setup/accel_calibration/widget.hpp"

#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hw
{
AccelCalibrationWidget::AccelCalibrationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : spinner_(Qt::WindowModal, this), thread_(node, bridge)
{
  const auto instruction = new qt::DescriptionWidget(
    "Press \"Start\" button with the flight controller\'s TOP surface facing up.\n\n", kBodyPSize);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);

  // TODO: Rvizで重力方向と測定結果を表示

  // Layout
  rows_->addWidget(instruction);
  rows_->addWidget(start_button_);
  rows_->addStretch();

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(&thread_, &AccelCalibrationThread::finished, this, &self::onCalibrationFinished);
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
}

const char* AccelCalibrationWidget::name() const
{
  return "Accelerometer\nCalibration";
}

const char* AccelCalibrationWidget::title() const
{
  return "Calibrate Accelerometer";
}

void AccelCalibrationWidget::reset()
{
  arming_.reset();
}

void AccelCalibrationWidget::setNamespace(const std::string& ns)
{
  reset();

  thread_.setNamespace(ns);
}

void AccelCalibrationWidget::onStartButtonClicked()
{
  // アームされていないことを確認
  if (!arming_) {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status is not received yet.");
    return;
  }
  if (arming_->data) {
    qt::qWarnBox(this, "This operation cannot be performed while the rotors are armed.");
    return;
  }

  spinner_.show();
  spinner_.start();

  thread_.start();
}

void AccelCalibrationWidget::onCalibrationFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (success) {
    qt::qInfoBox(this, message);
  }
  else {
    qt::qErrorBox(this, message);
  }
}

void AccelCalibrationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}
}  // namespace hw
}  // namespace gui
