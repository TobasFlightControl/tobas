#include "tobas_control_system/rotors_viewer/rotors_viewer.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace gui
{
namespace gcs
{
RotorsViewerWiddget::RotorsViewerWiddget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone)
{
  cols_ = new QHBoxLayout();
  setLayout(cols_);

  connect(this, &self::rotorStatesReceived, this, &self::rotorStateCbQt, Qt::QueuedConnection);
  connect(this, &self::rotorLivelinessReceived, this, &self::rotorLivelinessCbQt, Qt::QueuedConnection);
}

void RotorsViewerWiddget::reset()
{
  for (const auto& [link_name, meter] : meters_) {
    setSpeed(link_name, 0.);
    meter->setBackgroundColor(kAliveBackgroundColor);
  }
}

void RotorsViewerWiddget::updateInternalDataStructures()
{
  meters_.clear();
  qt::clearLayout(cols_);

  for (const auto& [link_name, rotor] : drone_.prop->rotors) {
    const auto meter = new SpeedmeterWidget();
    meter->setMaximumValue(tobas_std::rps2rpm(drone_.prop->maxSpeed(link_name)));
    meter->setTopText(QString::fromStdString(link_name));

    meters_[link_name] = meter;
    cols_->addWidget(meter);
  }

  reset();

  rotor_states_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kRotorStatesTopic), &self::rotorStatesCbRos, this);
  rotor_liveliness_sub_ = ros2::createSubscriber(
    node_,
    path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kRotorLivelinessesTopic),
    &self::rotorLivelinessCbRos,
    this);
}

void RotorsViewerWiddget::setSpeed(const std::string& link_name, const double& rps)
{
  const auto& meter = meters_.at(link_name);
  const auto rpm = static_cast<int>(tobas_std::rps2rpm(rps));
  meter->setValue(rpm);
  meter->setBottomText(bottomText(rpm));
}

void RotorsViewerWiddget::rotorStatesCbRos(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg)
{
  for (const auto& elem : msg->states) {
    if (!meters_.contains(elem.link_name)) {
      continue;
    }

    if (elem.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE) {
      continue;
    }

    Q_EMIT rotorStatesReceived(QString::fromStdString(elem.link_name), elem.speed);
  }
}

void RotorsViewerWiddget::rotorLivelinessCbRos(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& msg)
{
  for (const auto& elem : msg->data) {
    if (!meters_.contains(elem.link_name)) {
      continue;
    }

    Q_EMIT rotorLivelinessReceived(QString::fromStdString(elem.link_name), elem.alive);
  }
}

QString RotorsViewerWiddget::bottomText(int rpm)
{
  return QString::number(rpm) + " RPM";
}

void RotorsViewerWiddget::rotorStateCbQt(const QString& link_name, double speed)
{
  setSpeed(link_name.toStdString(), speed);
}

void RotorsViewerWiddget::rotorLivelinessCbQt(const QString& link_name, bool alive)
{
  const auto& meter = meters_.at(link_name.toStdString());

  if (alive) {
    meter->setBackgroundColor(kAliveBackgroundColor);
  }
  else {
    meter->setBackgroundColor(kDeadBackgroundColor);
  }
}
}  // namespace gcs
}  // namespace gui
