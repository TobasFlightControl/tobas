#include "tobas_sensor_calibration/mag_calibration/mag_calibration.hpp"

#include <QRadioButton>

#include <tobas_qt_tools/cast.hpp>

#include "tobas_sensor_calibration/mag_calibration/complete/complete.hpp"
#include "tobas_sensor_calibration/mag_calibration/large_vehicle/large_vehicle.hpp"

namespace gui
{
namespace sc
{
MagCalibrationWidget::MagCalibrationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
{
  btn_group_ = new QButtonGroup(this);
  btn_group_->setExclusive(true);

  stack_ = new qt::StackedWidget();

  int id = 0;
  addMagCalibWidget(new CompleteMagCalibWidget(node, bridge), "Complete Calibration", id++);
  addMagCalibWidget(new LargeVehicleMagCalibWidget(node, bridge), "Large Vehicle Calibration", id++);

  setCurrentIndex(kDefaultIndex);

  rows_->addSpacing(20);
  rows_->addWidget(stack_);

  connect(btn_group_, &QButtonGroup::idClicked, stack_, &qt::StackedWidget::setCurrentIndex);
}

const char* MagCalibrationWidget::title() const
{
  return "Calibrate Magnetometer";
}

void MagCalibrationWidget::reset()
{
  for (int i = 0; i < stack_->count(); ++i) {
    const auto widget = getWidget(i);
    widget->reset();
  }
}

void MagCalibrationWidget::setNamespace(const std::string& ns)
{
  for (int i = 0; i < stack_->count(); ++i) {
    const auto widget = getWidget(i);
    widget->reset();
    widget->setNamespace(ns);
  }

  setCurrentIndex(kDefaultIndex);
}

void MagCalibrationWidget::addMagCalibWidget(BaseMagCalibWidget* widget, const QString& label, int id)
{
  const auto btn = new QRadioButton(label);
  btn_group_->addButton(btn, id++);
  rows_->addWidget(btn);
  stack_->addWidget(widget);
}

BaseMagCalibWidget* MagCalibrationWidget::getWidget(int index)
{
  return qt::qPointerCast<BaseMagCalibWidget>(stack_->widget(index));
}

void MagCalibrationWidget::setCurrentIndex(int index)
{
  btn_group_->button(index)->setChecked(true);
  stack_->setCurrentIndex(index);
}
}  // namespace sc
}  // namespace gui
