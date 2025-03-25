#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>

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
  tabs_->ignoreWheelEvent();  // 不可視なタブのウィジェットを表示しないように
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

void HardwareSetupWidget::reset()
{
  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto widget = qt::qPointerCast<BaseHardwareSetupWidget>(tabs_->widget(i));
    widget->reset();
  }
}

void HardwareSetupWidget::updateInternalDataStructures()
{
  reset();

  accel_calib_->setNamespace(drone_.name);
  mag_calib_->setNamespace(drone_.name);
  rcin_calib_->setNamespace(drone_.name);
  rotor_test_->updateInternalDataStructures();
  joint_test_->updateInternalDataStructures();
}
}  // namespace hw
}  // namespace gui
