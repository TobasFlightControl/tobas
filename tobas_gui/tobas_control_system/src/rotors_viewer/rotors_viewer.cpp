#include "tobas_control_system/rotors_viewer/rotors_viewer.hpp"

#include <tobas_qt_tools/util.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace gui
{
namespace gcs
{
RotorsViewerWiddget::RotorsViewerWiddget(const RosQtBridge& bridge, const tobas::Drone& drone) : drone_(drone)
{
  cols_ = new QHBoxLayout();
  setLayout(cols_);

  connect(&bridge, &RosQtBridge::rotorStatesReceived, this, &self::rotorStatesCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::rotorLivelinessesReceived, this, &self::rotorLivelinessCb, Qt::QueuedConnection);
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
  for (const auto& elem : msg->states) {
    if (!meters_.contains(elem.link_name)) {
      continue;
    }

    if (elem.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE) {
      continue;
    }

    setSpeed(elem.link_name, elem.speed);
  }
}

void RotorsViewerWiddget::rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& msg)
{
  for (const auto& elem : msg->data) {
    if (!meters_.contains(elem.link_name)) {
      continue;
    }

    const auto& meter = meters_.at(elem.link_name);

    if (elem.alive) {
      meter->setBackgroundColor(kAliveBackgroundColor);
    }
    else {
      meter->setBackgroundColor(kDeadBackgroundColor);
    }
  }
}

QString RotorsViewerWiddget::bottomText(int rpm)
{
  return QString::number(rpm) + " RPM";
}
}  // namespace gcs
}  // namespace gui
