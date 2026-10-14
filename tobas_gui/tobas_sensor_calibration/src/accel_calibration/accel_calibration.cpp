// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/accel_calibration/accel_calibration.hpp"

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_ros2_tools/time.hpp>

namespace tobas::gui::sc
{
AccelCalibrationWidget::AccelCalibrationWidget(const rqt::RosQtBridge& _bridge)
  : spinner_(Qt::WindowModal, this), thread_(_bridge)
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Place the flight controller on a level surface. The yellow Raw marker is a rough guide before calibration.\n\n"
    "2. Click 'Start'. Calibration will complete in a few seconds.\n\n"
    "3. Confirm that the green Calibrated marker appears near the center.\n\n",
    cmn::kBodyPSize);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(100, 40);

  level_indicator_ = new LevelIndicatorWidget();

  // Layout
  rows_->addWidget(instruction);
  rows_->addWidget(start_button_);
  rows_->addWidget(level_indicator_, Qt::AlignLeft);
  rows_->addStretch();

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(&_bridge, &rqt::RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
  connect(&_bridge, &rqt::RosQtBridge::rawImuReceived, this, &self::rawImuCb, Qt::QueuedConnection);
  connect(&_bridge, &rqt::RosQtBridge::imuReceived, this, &self::calibratedImuCb, Qt::QueuedConnection);
}

const char* AccelCalibrationWidget::title() const
{
  return "Calibrate Accelerometer";
}

void AccelCalibrationWidget::reset()
{
  thread_.reset();
  level_indicator_->clear();

  arming_.reset();
  imu_raw_.reset();
  imu_calib_.reset();
}

void AccelCalibrationWidget::updateInternalDataStructures()
{
}

void AccelCalibrationWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns)
{
  thread_.initializeRosInterfaces(std::move(_node), _ns);
}

void AccelCalibrationWidget::clearRosInterfaces()
{
  thread_.clearRosInterfaces();
}

void AccelCalibrationWidget::onStartButtonClicked()
{
  // Confirm that the vehicle is not armed.
  if (!arming_) {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status has not been received yet.");
    return;
  }
  if (arming_->data) {
    qt::qWarnBox(this, "This operation cannot be performed while the vehicle is armed.");
    return;
  }

  spinner_.start();
  const auto [success, message] = qt::startThreadAndWait(thread_, &AccelCalibrationThread::finished);
  spinner_.stop();

  if (success) {
    qt::qInfoBox(this, message);
  }
  else {
    qt::qErrorBox(this, message);
  }
}

void AccelCalibrationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming)
{
  arming_ = _arming;
}

void AccelCalibrationWidget::rawImuCb(const tobas_msgs::Imu::ConstSharedPtr& _imu_raw)
{
  const auto dt = imu_raw_ ? (_imu_raw->header.stamp - imu_raw_->header.stamp).seconds() : 0.0;
  imu_raw_ = _imu_raw;
  level_indicator_->setRawAccel(_imu_raw->accel, dt);
}

void AccelCalibrationWidget::calibratedImuCb(const tobas_msgs::Imu::ConstSharedPtr& _imu_calib)
{
  const auto dt = imu_calib_ ? (_imu_calib->header.stamp - imu_calib_->header.stamp).seconds() : 0.0;
  imu_calib_ = _imu_calib;
  level_indicator_->setCalibratedAccel(_imu_calib->accel, dt);
}
}  // namespace tobas::gui::sc
