#include <QVBoxLayout>

#include "tobas_hardware_setup/hardware_setup.hpp"

namespace gui
{
namespace hw
{
HardwareSetupWidget::HardwareSetupWidget(rclcpp::Node::SharedPtr node, const kdl::Tree& tree, const tobas::Drone& drone)
  : drone_(drone)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  tabs_ = new qt::VerticalTabWidget();
  rows->addWidget(tabs_);

  network_setting_ = new NetworkSettingWidget(node);
  accel_calib_ = new AccelCalibrationWidget(node);
  mag_calib_ = new MagCalibrationWidget(node);
  rcin_calib_ = new RCInputCalibrationWidget(node);
  rotor_test_ = new RotorTestWidget(node, drone);
  joint_test_ = new JointTestWidget(node, tree, drone);

  tabs_->addTab(network_setting_, network_setting_->name());
  tabs_->addTab(accel_calib_, accel_calib_->name());
  tabs_->addTab(mag_calib_, mag_calib_->name());
  tabs_->addTab(rcin_calib_, rcin_calib_->name());
  tabs_->addTab(rotor_test_, rotor_test_->name());
  tabs_->addTab(joint_test_, joint_test_->name());

  tabs_->setTabSize(kTabWidth, kTabHeight);
}

void HardwareSetupWidget::updateInternalDataStructures()
{
  accel_calib_->setNamespace(drone_.name);
  mag_calib_->setNamespace(drone_.name);
  rcin_calib_->setNamespace(drone_.name);
  rotor_test_->updateInternalDataStructures();
  joint_test_->updateInternalDataStructures();
}

void HardwareSetupWidget::resetTime()
{
  mag_calib_->resetTime();
}
}  // namespace hw
}  // namespace gui
