// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/sensor_calibration.hpp"

#include <QLabel>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/widgets/widget.hpp>

namespace tobas::gui::sc
{
SensorCalibrationWidget::SensorCalibrationWidget(const rqt::RosQtBridge& _bridge, const Drone& _drone) : drone_(_drone)
{
  setTabSize(64, 32);
  enableWheelEvent(false);

  accel_calib_ = new AccelCalibrationWidget(_bridge);
  addTab(createTabPage(accel_calib_), "Accelerometer");

  mag_calib_ = new MagCalibrationWidget(_bridge);
  addTab(createTabPage(mag_calib_), "Magnetometer");

  rcin_calib_ = new RCInputCalibrationWidget(_bridge, _drone);
  addTab(createTabPage(rcin_calib_), "Radio Control");

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

void SensorCalibrationWidget::initializeRosInterfaces(
  rclcpp::Node::SharedPtr _node,
  const std::string& _ns,
  bool _simulation)
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->initializeRosInterfaces(_node, _ns, _simulation);
  }

  setTabsEnabled(true);
}

BaseWidget* SensorCalibrationWidget::getWidget(int _index)
{
  return widget(_index)->findChild<BaseWidget*>(QString(), Qt::FindDirectChildrenOnly);
}

const BaseWidget* SensorCalibrationWidget::getWidget(int _index) const
{
  return widget(_index)->findChild<BaseWidget*>(QString(), Qt::FindDirectChildrenOnly);
}

QWidget* SensorCalibrationWidget::createTabPage(BaseWidget* _content)
{
  const auto page = new qt::Widget();
  page->setAutoFillBackground(true);

  const auto title = new QLabel(_content->title());
  title->setFont(qt::DefaultFont(cmn::kTitlePointSize, QFont::Bold));

  _content->layout()->setContentsMargins(0, 0, 0, 0);

  const auto rows = new QVBoxLayout(page);
  rows->addWidget(title, 0, Qt::AlignTop);
  rows->addSpacing(16);
  rows->addWidget(_content, 1);

  return page;
}

void SensorCalibrationWidget::setTabsEnabled(bool _enabled)
{
  for (int i = 0; i < count(); ++i) {
    widget(i)->setEnabled(_enabled);
  }
}

void SensorCalibrationWidget::setCompleted(int _index)
{
  setTabBackgroundColor(_index, Qt::green);
}

void SensorCalibrationWidget::setCompleted(BaseWidget* _widget)
{
  setCompleted(indexOf(_widget->parentWidget()));
}

void SensorCalibrationWidget::setIncompleted(int _index)
{
  setTabBackgroundColor(_index, Qt::red);
}

void SensorCalibrationWidget::setIncompleted(BaseWidget* _widget)
{
  setIncompleted(indexOf(_widget->parentWidget()));
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
}  // namespace tobas::gui::sc
