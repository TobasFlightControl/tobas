#include <format>

#include <tobas_math/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "tobas_control_system/power_source_viewer/battery_viewer.hpp"

namespace gui
{
namespace gcs
{
BatteryViewerWidget::BatteryViewerWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone)
{
  voltage_ = new qt::HPositionBarWidget();
  current_ = new qt::HPositionBarWidget();

  voltage_->setFixedHeight(kBarHeight);
  current_->setFixedHeight(kBarHeight);

  // Layout
  const auto form = new qt::FormLayout();
  form->addVAlignedRow(new qt::Label("Battery Voltage", kLabelPSize), voltage_);
  form->addVAlignedRow(new qt::Label("Battery Current", kLabelPSize), current_);
  setLayout(form);
}

void BatteryViewerWidget::reset()
{
  voltage_->setUpper(voltage_->getMinimum());
  voltage_->setText("");

  current_->setUpper(current_->getMinimum());
  current_->setText("");
}

void BatteryViewerWidget::updateInternalDataStructures()
{
  reset();

  if (drone_.prop->type() == tobas::propulsion_system_t::ELECTRIC)
  {
    eprop_ = boost::polymorphic_pointer_downcast<tobas::ElectricPropulsionSystemConfig>(drone_.prop);

    voltage_->setLower(eprop_->battery.sag_voltage);
    voltage_->setMinimum(eprop_->battery.sag_voltage);
    voltage_->setMaximum(eprop_->battery.max_voltage);

    current_->setLower(0.);
    current_->setMinimum(0.);
    current_->setMaximum(eprop_->battery.max_current);

    battery_sub_ = ros2::createSubscriber(
      node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kBatteryTopic), &self::batteryCb, this);
  }
  else
  {
    eprop_ = nullptr;
    battery_sub_ = nullptr;
  }
}

void BatteryViewerWidget::updateVoltage(const double& voltage)
{
  const auto volt_rate = math::remap(voltage, eprop_->battery.sag_voltage, eprop_->battery.max_voltage, 0., 100.);
  voltage_->setUpper(voltage);
  voltage_->setText(std::format("{:.2f} V ({:.0f} %)", voltage, volt_rate).c_str());

  if (volt_rate > 20.)
    voltage_->setFillColor(Qt::green);
  else if (volt_rate > 10.)
    voltage_->setFillColor(Qt::yellow);
  else
    voltage_->setFillColor(Qt::red);
}

void BatteryViewerWidget::updateCurrent(const double& current)
{
  current_->setUpper(current);
  current_->setText(std::format("{:.2f} A", current).c_str());

  if (current < eprop_->battery.max_current * 0.6)
    current_->setFillColor(Qt::green);
  else if (current < eprop_->battery.max_current * 0.8)
    current_->setFillColor(Qt::yellow);
  else
    current_->setFillColor(Qt::red);
}

void BatteryViewerWidget::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  updateVoltage(battery->voltage);
  updateCurrent(battery->current);
}
}  // namespace gcs
}  // namespace gui
