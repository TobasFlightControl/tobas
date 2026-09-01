// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/sensor_calibration.hpp"

#include <tobas_qt_tools/cast.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
namespace
{
// Without at least this much height, the `TabBar` text is clipped horizontally for some reason.
constexpr int kTabHeight = 35;
constexpr int kTabWidth = 70;
}  // namespace

SensorCalibrationWidget::SensorCalibrationWidget(const rqt::RosQtBridge& bridge, const Drone& drone) : drone_(drone)
{
  setTabSize(kTabWidth, kTabHeight);
  enableWheelEvent(false);

  accel_calib_ = new AccelCalibrationWidget(bridge);
  addTab(accel_calib_, "Accelerometer");

  mag_calib_ = new MagCalibrationWidget(bridge);
  addTab(mag_calib_, "Magnetometer");

  rcin_calib_ = new RCInputCalibrationWidget(bridge, drone);
  addTab(rcin_calib_, "Radio Control");

  setTabsEnabled(false);

  // Connection
  connect(&bridge, &rqt::RosQtBridge::imuReceived, this, &self::imuCb, Qt::QueuedConnection);
  connect(&bridge, &rqt::RosQtBridge::magReceived, this, &self::magCb, Qt::QueuedConnection);
  connect(&bridge, &rqt::RosQtBridge::rcInputReceived, this, &self::rcInputCb, Qt::QueuedConnection);
}

void SensorCalibrationWidget::reset()
{
  for (int i = 0; i < count(); ++i) {
    setIncompleted(i);
    getWidget(i)->reset();
  }
}

void SensorCalibrationWidget::updateInternalDataStructures()
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->updateInternalDataStructures();
  }
}

void SensorCalibrationWidget::clearRosInterfaces()
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->clearRosInterfaces();
  }

  setTabsEnabled(false);
}

void SensorCalibrationWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->initializeRosInterfaces(node, ns);
  }

  setTabsEnabled(true);
}

BaseWidget* SensorCalibrationWidget::getWidget(int index)
{
  return qt::qPointerCast<BaseWidget>(widget(index));
}

const BaseWidget* SensorCalibrationWidget::getWidget(int index) const
{
  return qt::qConstPointerCast<BaseWidget>(widget(index));
}

void SensorCalibrationWidget::setTabsEnabled(bool enabled)
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->setEnabled(enabled);
  }
}

void SensorCalibrationWidget::setCompleted(int index)
{
  setTabBackgroundColor(index, Qt::green);
}

void SensorCalibrationWidget::setCompleted(BaseWidget* widget)
{
  setCompleted(indexOf(widget));
}

void SensorCalibrationWidget::setIncompleted(int index)
{
  setTabBackgroundColor(index, Qt::red);
}

void SensorCalibrationWidget::setIncompleted(BaseWidget* widget)
{
  setIncompleted(indexOf(widget));
}

void SensorCalibrationWidget::imuCb(const tobas_msgs::Imu::ConstSharedPtr&)
{
  setCompleted(accel_calib_);
}

void SensorCalibrationWidget::magCb(const tobas_msgs::MagneticField::ConstSharedPtr&)
{
  setCompleted(mag_calib_);
}

void SensorCalibrationWidget::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& msg)
{
  if (msg->status == tobas_msgs::msg::RCInput::STATUS_OK) {
    setCompleted(rcin_calib_);
  }
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
