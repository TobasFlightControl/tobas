// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/mag_calibration/mag_calibration.hpp"

#include <QRadioButton>

#include <tobas_qt_tools/cast.hpp>

#include "tobas_sensor_calibration/mag_calibration/complete/complete.hpp"
#include "tobas_sensor_calibration/mag_calibration/large_vehicle/large_vehicle.hpp"

namespace tobas
{
namespace gui
{
namespace sc
{
namespace
{
constexpr int kDefaultIndex = 0;
}  // namespace

MagCalibrationWidget::MagCalibrationWidget(const rqt::RosQtBridge& _bridge)
{
  btn_group_ = new QButtonGroup(this);
  btn_group_->setExclusive(true);

  stack_ = new qt::StackedWidget();

  int id = 0;
  addMagCalibWidget(new CompleteMagCalibWidget(_bridge), "Complete Calibration (Recommended)", id++);
  addMagCalibWidget(new LargeVehicleMagCalibWidget(_bridge), "Large Vehicle Calibration", id++);

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

void MagCalibrationWidget::updateInternalDataStructures()
{
}

void MagCalibrationWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns)
{
  for (int i = 0; i < stack_->count(); ++i) {
    const auto widget = getWidget(i);
    widget->reset();
    widget->initializeRosInterfaces(_node, _ns);
  }

  setCurrentIndex(kDefaultIndex);
}

void MagCalibrationWidget::clearRosInterfaces()
{
  for (int i = 0; i < stack_->count(); ++i) {
    getWidget(i)->clearRosInterfaces();
  }
}

void MagCalibrationWidget::addMagCalibWidget(BaseMagCalibWidget* _widget, const QString& _label, int _id)
{
  const auto btn = new QRadioButton(_label);
  btn_group_->addButton(btn, _id++);
  rows_->addWidget(btn);
  stack_->addWidget(_widget);
}

BaseMagCalibWidget* MagCalibrationWidget::getWidget(int _index)
{
  return qt::qPointerCast<BaseMagCalibWidget>(stack_->widget(_index));
}

void MagCalibrationWidget::setCurrentIndex(int _index)
{
  btn_group_->button(_index)->setChecked(true);
  stack_->setCurrentIndex(_index);
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
