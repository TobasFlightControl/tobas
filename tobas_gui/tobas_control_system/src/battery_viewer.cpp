#include <format>

#include <tobas_math/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "tobas_control_system/battery_viewer.hpp"

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

void BatteryViewerWidget::updateInternalDataStructures()
{
  voltage_->clear();
  voltage_->setLower(drone_.battery.sag_voltage);
  voltage_->setMinimum(drone_.battery.sag_voltage);
  voltage_->setMaximum(drone_.battery.max_voltage);

  current_->clear();
  current_->setLower(0.);
  current_->setMinimum(0.);
  current_->setMaximum(drone_.battery.max_current);

  batt_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kBatteryTopic), &self::battCb, this);
}

void BatteryViewerWidget::battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& batt)
{
  const auto volt_rate = math::remap(batt->voltage, drone_.battery.sag_voltage, drone_.battery.max_voltage, 0., 100.);
  voltage_->setUpper(batt->voltage);
  voltage_->setText(std::format("{:.2f} V ({:.0f} %)", batt->voltage, volt_rate).c_str());

  current_->setUpper(batt->current);
  current_->setText(std::format("{:.2f} A", batt->current).c_str());

  if (volt_rate < 10.)
    voltage_->setFillColor(Qt::red);
  else if (volt_rate < 20.)
    voltage_->setFillColor(Qt::yellow);
  else
    voltage_->setFillColor(Qt::green);
}
}  // namespace gcs
}  // namespace gui
