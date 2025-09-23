#include "tobas_sensor_calibration/mag_calibration/large_vehicle/large_vehicle.hpp"

#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

namespace gui
{
namespace sc
{
LargeVehicleMagCalibWidget::LargeVehicleMagCalibWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : spinner_(Qt::WindowModal, this), thread_(node, bridge)
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Make sure the GNSS is fixed. This is required to obtain the reference geomagnetic field.\n\n"
    "2. Place the flight controller on a level surface.\n\n"
    "3. Point the front of the flight controller as precisely as possible toward true north.\n\n"
    "4. Click \"Start\". Calibration will complete in a few seconds.\n\n",
    cmn::kBodyPSize);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);

  // Layout
  const auto rows = new QVBoxLayout();
  setLayout(rows);
  rows->addWidget(instruction);
  rows->addWidget(start_button_);
  rows->addStretch();

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(&thread_, &LargeVehicleMagCalibThread::finished, this, &self::onCalibrationFinished);
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
}

void LargeVehicleMagCalibWidget::reset()
{
  arming_.reset();
}

void LargeVehicleMagCalibWidget::setNamespace(const std::string& ns)
{
  thread_.setNamespace(ns);
}

void LargeVehicleMagCalibWidget::onStartButtonClicked()
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

void LargeVehicleMagCalibWidget::onCalibrationFinished(bool success, const QString& message)
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

void LargeVehicleMagCalibWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}
}  // namespace sc
}  // namespace gui
