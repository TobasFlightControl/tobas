#include <QVBoxLayout>

#include "tobas_hardware_setup/hardware_setup.hpp"

namespace gui
{
namespace hardware_setup
{
HardwareSetupWidget::HardwareSetupWidget(
  rclcpp::Node::SharedPtr node,
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  tabs_ = new qt::VerticalTabWidget();
  rows->addWidget(tabs_);

  accel_calib_ = new AccelCalibrationWidget(node);
  // TODO

  tabs_->addTab(accel_calib_, accel_calib_->name());

  tabs_->setMinimumHeight(kMinHeight);
  tabs_->setStyleSheet(
    QString::fromStdString(std::format("QTabBar::tab {{ height: {}px; width: {}px; }}", kTabHeight, kTabWidth)));
}
}  // namespace hardware_setup
}  // namespace gui
