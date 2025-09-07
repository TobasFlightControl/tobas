#include "tobas_sensor_calibration/sensor_calibration.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>

namespace gui
{
namespace sc
{
SensorCalibrationWidget::SensorCalibrationWidget(
  rclcpp::Node::SharedPtr node,
  const RosQtBridge& bridge,
  const tobas::Drone& drone)
  : drone_(drone)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  tabs_ = new qt::VerticalTabWidget();
  tabs_->enableWheelEvent(false);
  rows->addWidget(tabs_);

  accel_calib_ = new AccelCalibrationWidget(node, bridge);
  mag_calib_ = new MagCalibrationWidget(node, bridge);
  rcin_calib_ = new RCInputCalibrationWidget(node, bridge, drone);

  tabs_->addTab(accel_calib_, accel_calib_->name());
  tabs_->addTab(mag_calib_, mag_calib_->name());
  tabs_->addTab(rcin_calib_, rcin_calib_->name());

  tabs_->setTabSize(kTabWidth, kTabHeight);

  reset();

  // プロジェクトが読み込まれるまでは無効化
  setTabsEnabled(false);
}

void SensorCalibrationWidget::reset()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    setIncompleted(i);
    getWidget(i)->reset();
  }
}

void SensorCalibrationWidget::updateInternalDataStructures()
{
  reset();

  accel_calib_->setNamespace(drone_.name);
  mag_calib_->setNamespace(drone_.name);
  rcin_calib_->updateInternalDataStructures();

  // 各タブを有効化
  setTabsEnabled(true);

  // タブを表示・非表示した際の歪みを整える
  tabs_->update();
}

BaseWidget* SensorCalibrationWidget::getWidget(int index)
{
  return qt::qPointerCast<BaseWidget>(tabs_->widget(index));
}

void SensorCalibrationWidget::setTabsEnabled(bool enabled)
{
  for (int i = 0; i < tabs_->count(); ++i) {
    getWidget(i)->setEnabled(enabled);
  }
}

void SensorCalibrationWidget::setCompleted(int index)
{
  tabs_->setTabBackgroundColor(index, Qt::green);
}

void SensorCalibrationWidget::setIncompleted(int index)
{
  tabs_->setTabBackgroundColor(index, Qt::red);
}
}  // namespace sc
}  // namespace gui
