#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_control_system/rotors_viewer/rotors_viewer.hpp"

namespace gui
{
namespace gcs
{
RotorsViewerWiddget::RotorsViewerWiddget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone)
{
  cols_ = new QHBoxLayout();
  setLayout(cols_);
}

void RotorsViewerWiddget::reset()
{
  for (const auto& [channel, meter] : meters_)
  {
    setSpeed(channel, 0.);
    meter->setBackgroundColor(kAliveBackgroundColor);
  }
}

void RotorsViewerWiddget::updateInternalDataStructures()
{
  meters_.clear();
  qt::clearLayout(cols_);

  for (const auto& [link_name, rotor] : drone_.prop->rotors)
  {
    const auto meter = new SpeedmeterWidget();
    meter->setMaximumValue(tobas_std::rps2rpm(drone_.prop->maxSpeed(link_name)));
    meter->setTopText(QString::fromStdString(link_name));

    meters_[link_name] = meter;
    cols_->addWidget(meter);
  }

  reset();

  rotor_states_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kRotorStatesTopic), &self::rotorStatesCb, this);
  rotor_liveliness_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kRotorLivelinessTopic), &self::rotorLivelinessCb,
    this);
}

void RotorsViewerWiddget::setSpeed(const std::string& link_name, const double& rps)
{
  const auto& meter = meters_.at(link_name);
  const auto rpm = static_cast<int>(tobas_std::rps2rpm(rps));
  meter->setValue(rpm);
  meter->setBottomText(bottomText(rpm));
}

void RotorsViewerWiddget::rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg)
{
  for (const auto& state : msg->states)
  {
    if (!meters_.contains(state.link_name))
      continue;

    if (state.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE)
      continue;

    setSpeed(state.link_name, state.speed);
  }
}

void RotorsViewerWiddget::rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& msg)
{
  for (const auto& liveliness : msg->data)
  {
    if (!meters_.contains(liveliness.link_name))
      continue;

    const auto& meter = meters_.at(liveliness.link_name);

    if (liveliness.alive)
      meter->setBackgroundColor(kAliveBackgroundColor);
    else
      meter->setBackgroundColor(kDeadBackgroundColor);
  }
}

QString RotorsViewerWiddget::bottomText(int rpm)
{
  return QString::number(rpm) + " RPM";
}
}  // namespace gcs
}  // namespace gui
