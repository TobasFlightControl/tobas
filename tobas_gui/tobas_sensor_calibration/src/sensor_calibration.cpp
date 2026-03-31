#include "tobas_sensor_calibration/sensor_calibration.hpp"

#include <tobas_qt_tools/cast.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
SensorCalibrationWidget::SensorCalibrationWidget(
  rclcpp::Node::SharedPtr node,
  const RosQtBridge& bridge,
  const Drone& drone)
  : drone_(drone)
{
  setTabSize(kTabWidth, kTabHeight);
  enableWheelEvent(false);

  accel_calib_ = new AccelCalibrationWidget(node, bridge);
  addTab(accel_calib_, "Accelerometer");

  mag_calib_ = new MagCalibrationWidget(node, bridge);
  addTab(mag_calib_, "Magnetometer");

  rcin_calib_ = new RCInputCalibrationWidget(node, bridge, drone);
  addTab(rcin_calib_, "Radio Control");

  setTabsEnabled(false);

  // Connection
  connect(&bridge, &RosQtBridge::imuReceived, this, &self::imuCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::magReceived, this, &self::magCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::rcInputReceived, this, &self::rcInputCb, Qt::QueuedConnection);
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
  reset();

  const auto ns = '/' + drone_.name;

  accel_calib_->setNamespace(ns);
  mag_calib_->setNamespace(ns);
  rcin_calib_->updateInternalDataStructures();

  // 各タブを有効化
  setTabsEnabled(true);

  // タブを表示・非表示した際の歪みを整える
  update();
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
  if (msg->ok) {
    setCompleted(rcin_calib_);
  }
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
