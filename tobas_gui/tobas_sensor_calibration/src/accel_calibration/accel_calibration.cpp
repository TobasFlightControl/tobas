#include "tobas_sensor_calibration/accel_calibration/accel_calibration.hpp"

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
AccelCalibrationWidget::AccelCalibrationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : spinner_(Qt::WindowModal, this), thread_(node, bridge)
{
  const auto instruction = new tobas::qt::DescriptionWidget(
    "1. Place the flight controller on a level surface.\n\n"
    "2. Click \"Start\". Calibration will complete in a few seconds.\n\n",
    cmn::kBodyPSize);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);

  // TODO: Rvizで重力方向と測定結果を表示

  // Layout
  rows_->addWidget(instruction);
  rows_->addWidget(start_button_);
  rows_->addStretch();

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
}

const char* AccelCalibrationWidget::title() const
{
  return "Calibrate Accelerometer";
}

void AccelCalibrationWidget::reset()
{
  thread_.reset();

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
    tobas::qt::qWarnBox(this, "This operation cannot be performed because the arming status has not been received yet.");
    return;
  }
  if (arming_->data) {
    tobas::qt::qWarnBox(this, "This operation cannot be performed while the vehicle is armed.");
    return;
  }

  spinner_.start();
  const auto [success, message] = tobas::qt::startThreadAndWait(thread_, &AccelCalibrationThread::finished);
  spinner_.stop();

  if (success) {
    tobas::qt::qInfoBox(this, message);
  }
  else {
    tobas::qt::qErrorBox(this, message);
  }
}

void AccelCalibrationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
