#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_control_system/rotors_viewer/rotors_viewer.hpp"

namespace gui
{
namespace control_system
{
RotorsViewerWiddget::RotorsViewerWiddget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone)
{
  cols_ = new QHBoxLayout();
  setLayout(cols_);
}

void RotorsViewerWiddget::updateInternalDataStructures()
{
  speeds_sub_ = nullptr;

  meters_.clear();
  qt::clearLayout(cols_);

  for (const auto& rotor : drone_.rotors)
  {
    const auto meter = new SpeedmeterWidget();
    meter->setMaximumValue(tobas_std::rps2rpm(rotor.max_rot_speed));
    meter->setTopText(rotor.link_name.c_str());
    meter->setBottomText(bottomText(0));

    meters_.push_back(meter);
    cols_->addWidget(meter);
  }

  speeds_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kRotorSpeedsTopic), &self::speedsCb, this);
}

void RotorsViewerWiddget::speedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& speeds)
{
  for (const auto& speed : speeds->speeds)
  {
    if (speed.channel >= meters_.size())
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Channel " << (int)speed.channel << " is out of range.");
      return;
    }

    const auto speed_rpm = static_cast<int>(tobas_std::rps2rpm(speed.speed));
    meters_.at(speed.channel)->setValue(speed_rpm);
    meters_.at(speed.channel)->setBottomText(bottomText(speed_rpm));
  }
}

QString RotorsViewerWiddget::bottomText(int rpm)
{
  return QString::number(rpm) + " RPM";
}
}  // namespace control_system
}  // namespace gui
