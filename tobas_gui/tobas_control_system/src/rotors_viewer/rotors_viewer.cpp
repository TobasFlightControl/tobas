// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/rotors_viewer/rotors_viewer.hpp"

#include <tobas_qt_tools/util.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas::gui::ctrl
{
namespace
{
constexpr char kAliveBackgroundColor[] = "transparent";
constexpr char kDeadBackgroundColor[] = "red";
}  // namespace

RotorsViewerWiddget::RotorsViewerWiddget(const rqt::RosQtBridge& _bridge, const Drone& _drone) : drone_(_drone)
{
  cols_ = new QHBoxLayout(this);

  connect(&_bridge, &rqt::RosQtBridge::rotorStatesReceived, this, &self::rotorStatesCb, Qt::QueuedConnection);
  connect(&_bridge, &rqt::RosQtBridge::rotorLivelinessReceived, this, &self::rotorLivelinessCb, Qt::QueuedConnection);
}

void RotorsViewerWiddget::reset()
{
  for (const auto& [link_name, meter] : meters_) {
    setSpeed(link_name, 0.0);
    meter->setBackgroundColor(kAliveBackgroundColor);
  }
}

void RotorsViewerWiddget::updateInternalDataStructures()
{
  meters_.clear();
  qt::clearLayout(cols_);

  for (const auto& [link_name, rotor] : drone_.prop->rotors) {
    const auto meter = new SpeedmeterWidget();
    meter->setMaximumValue(st::rps2rpm(drone_.prop->maxSpeed(link_name)));
    meter->setTopText(QString::fromStdString(link_name));

    meters_[link_name] = meter;
    cols_->addWidget(meter);
  }
}

void RotorsViewerWiddget::setSpeed(const std::string& _link_name, double _rps)
{
  const auto& meter = meters_.at(_link_name);
  const auto rpm = static_cast<int>(st::rps2rpm(_rps));
  meter->setValue(rpm);
  meter->setBottomText(bottomText(rpm));
}

void RotorsViewerWiddget::rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& _msg)
{
  for (const auto& elem : _msg->states) {
    if (!meters_.contains(elem.link_name)) {
      continue;
    }

    if (elem.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE) {
      continue;
    }

    setSpeed(elem.link_name, elem.speed);
  }
}

void RotorsViewerWiddget::rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& _msg)
{
  for (const auto& elem : _msg->data) {
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

QString RotorsViewerWiddget::bottomText(int _rpm)
{
  return QString::number(_rpm) + " RPM";
}
}  // namespace tobas::gui::ctrl
