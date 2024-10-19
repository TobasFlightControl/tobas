#include <format>

#include <tobas_math/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "tobas_control_system/battery_cpu_viewer.hpp"

namespace gui
{
namespace control_system
{
BatteryCPUViewerWidget::BatteryCPUViewerWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone)
{
  batt_voltage_ = new qt::HPositionBarWidget();
  cpu_temp_ = new qt::HPositionBarWidget();
  cpu_load_ = new qt::HPositionBarWidget();

  batt_voltage_->setFixedHeight(kBarHeight);
  cpu_temp_->setFixedHeight(kBarHeight);
  cpu_load_->setFixedHeight(kBarHeight);

  // Layout
  const auto form = new qt::FormLayout();
  form->addVAlignedRow(new qt::Label("Battery Voltage", kLabelPSize), batt_voltage_);
  form->addVAlignedRow(new qt::Label("CPU Temperature", kLabelPSize), cpu_temp_);
  form->addVAlignedRow(new qt::Label("CPU Load", kLabelPSize), cpu_load_);

  setLayout(form);
}

void BatteryCPUViewerWidget::updateInternalDataStructures()
{
  batt_voltage_->clear();
  batt_voltage_->setLower(drone_.battery.sag_voltage);
  batt_voltage_->setMinimum(drone_.battery.sag_voltage);
  batt_voltage_->setMaximum(drone_.battery.max_voltage);

  cpu_temp_->clear();
  cpu_temp_->setLower(kMinCPUTemp);
  cpu_temp_->setMinimum(kMinCPUTemp);
  cpu_temp_->setMaximum(kMaxCPUTemp);

  cpu_load_->clear();
  cpu_load_->setLower(kMinCPULoad);
  cpu_load_->setMinimum(kMinCPULoad);
  cpu_load_->setMaximum(kMaxCPULoad);

  batt_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kBatteryTopic), &self::battCb, this);
  cpu_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kCPUTopic), &self::cpuCb, this);
}

void BatteryCPUViewerWidget::battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& batt)
{
  const auto rate = math::remap(batt->voltage, drone_.battery.sag_voltage, drone_.battery.max_voltage, 0., 100.);

  batt_voltage_->setUpper(batt->voltage);
  batt_voltage_->setText(std::format("{:.2f} V ({:.0f} %)", batt->voltage, rate).c_str());

  if (rate < 10.)
    batt_voltage_->setFillColor(Qt::red);
  else if (rate < 20.)
    batt_voltage_->setFillColor(Qt::yellow);
  else
    batt_voltage_->setFillColor(Qt::green);
}

void BatteryCPUViewerWidget::cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu)
{
  cpu_temp_->setUpper(cpu->temperature);
  cpu_temp_->setText(std::format("{:.0f} ℃", cpu->temperature).c_str());
  if (cpu->temperature > 85.)
    cpu_temp_->setFillColor(Qt::magenta);
  else if (cpu->temperature > 70.)
    cpu_temp_->setFillColor(Qt::red);
  else if (cpu->temperature > 50.)
    cpu_temp_->setFillColor(Qt::yellow);
  else
    cpu_temp_->setFillColor(Qt::green);

  cpu_load_->setUpper(cpu->load * 100);
  cpu_load_->setText(std::format("{:.0f} %", cpu->load * 100).c_str());
  if (cpu->load > 80.)
    cpu_load_->setFillColor(Qt::red);
  else if (cpu->load > 60.)
    cpu_load_->setFillColor(Qt::yellow);
  else
    cpu_load_->setFillColor(Qt::green);
}
}  // namespace control_system
}  // namespace gui
