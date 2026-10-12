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
SensorCalibrationWidget::SensorCalibrationWidget(const rqt::RosQtBridge& _bridge, const Drone& _drone) : drone_(_drone)
{
  // Without at least this much height, the `TabBar` text is clipped horizontally for some reason.
  setTabSize(70, 35);

  accel_calib_ = new AccelCalibrationWidget(_bridge);
  addTab(accel_calib_, "Accelerometer");

  mag_calib_ = new MagCalibrationWidget(_bridge);
  addTab(mag_calib_, "Magnetometer");

  rcin_calib_ = new RCInputCalibrationWidget(_bridge, _drone);
  addTab(rcin_calib_, "Radio Control");

  enableWheelEvent(false);
  setTabsEnabled(false);

  // Connection
  connect(&_bridge, &rqt::RosQtBridge::imuReceived, this, &self::imuCb, Qt::QueuedConnection);
  connect(&_bridge, &rqt::RosQtBridge::magReceived, this, &self::magCb, Qt::QueuedConnection);
  connect(&_bridge, &rqt::RosQtBridge::rcInputReceived, this, &self::rcInputCb, Qt::QueuedConnection);
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

void SensorCalibrationWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns)
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->initializeRosInterfaces(_node, _ns);
  }

  setTabsEnabled(true);
}

BaseWidget* SensorCalibrationWidget::getWidget(int _index)
{
  return qt::qPointerCast<BaseWidget>(widget(_index));
}

const BaseWidget* SensorCalibrationWidget::getWidget(int _index) const
{
  return qt::qConstPointerCast<BaseWidget>(widget(_index));
}

void SensorCalibrationWidget::setTabsEnabled(bool _enabled)
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->setEnabled(_enabled);
  }
}

void SensorCalibrationWidget::setCompleted(int _index)
{
  setTabBackgroundColor(_index, Qt::green);
}

void SensorCalibrationWidget::setCompleted(BaseWidget* _widget)
{
  setCompleted(indexOf(_widget));
}

void SensorCalibrationWidget::setIncompleted(int _index)
{
  setTabBackgroundColor(_index, Qt::red);
}

void SensorCalibrationWidget::setIncompleted(BaseWidget* _widget)
{
  setIncompleted(indexOf(_widget));
}

void SensorCalibrationWidget::imuCb(const tobas_msgs::Imu::ConstSharedPtr&)
{
  setCompleted(accel_calib_);
}

void SensorCalibrationWidget::magCb(const tobas_msgs::MagneticField::ConstSharedPtr&)
{
  setCompleted(mag_calib_);
}

void SensorCalibrationWidget::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& _msg)
{
  if (_msg->status == tobas_msgs::msg::RCInput::STATUS_OK) {
    setCompleted(rcin_calib_);
  }
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
