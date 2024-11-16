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
  rotor_states_sub_ = nullptr;

  meters_.clear();
  qt::clearLayout(cols_);

  for (const auto& [_, rotor] : drone_.rotors)
  {
    const auto meter = new SpeedmeterWidget();
    meter->setMaximumValue(tobas_std::rps2rpm(rotor.max_rot_speed));
    meter->setTopText(QString::fromStdString(rotor.link_name));
    meter->setBottomText(bottomText(0));

    meters_[rotor.channel] = meter;
    cols_->addWidget(meter);
  }

  rotor_states_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kRotorStatesTopic), &self::rotorStatesCb, this);
}

void RotorsViewerWiddget::rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& states)
{
  for (const auto& state : states->states)
  {
    if (state.status == tobas_msgs::msg::RotorState::NO_COMMUNICATION)
      continue;
    if (!meters_.contains(state.channel))
      continue;

    const auto speed_rpm = static_cast<int>(tobas_std::rps2rpm(state.speed));
    meters_.at(state.channel)->setValue(speed_rpm);
    meters_.at(state.channel)->setBottomText(bottomText(speed_rpm));
  }
}

QString RotorsViewerWiddget::bottomText(int rpm)
{
  return QString::number(rpm) + " RPM";
}
}  // namespace control_system
}  // namespace gui
