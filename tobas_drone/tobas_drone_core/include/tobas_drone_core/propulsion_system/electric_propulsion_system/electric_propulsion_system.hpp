#pragma once

#include <boost/polymorphic_pointer_cast.hpp>

#include "../propulsion_system.hpp"
#include "./battery.hpp"
#include "./electric_rotor.hpp"

namespace tobas
{
class ElectricPropulsionSystemConfig : public PropulsionSystemConfig
{
  static constexpr char kBatteryKey[] = "battery";

  static constexpr double kMinSpeed = tobas_std::rpm2rps(300);  // 静止摩擦を防ぐための最小回転数 [rad/s]

public:
  using SharedPtr = std::shared_ptr<ElectricPropulsionSystemConfig>;
  using ConstSharedPtr = std::shared_ptr<const ElectricPropulsionSystemConfig>;

  BatteryConfig battery;

  bool isValid() const override;

  bool load(const YAML::Node& node) override;
  YAML::Node dump() const override;

  propulsion_system_t type() const override;

  double minSpeed(const std::string& link_name) const override;
  double maxSpeed(const std::string& link_name) const override;
  double minThrust(const std::string& link_name) const override;
  double maxThrust(const std::string& link_name) const override;

  inline ElectricRotorConfig::SharedPtr getRotor(const std::string& link_name);
  inline ElectricRotorConfig::ConstSharedPtr getRotor(const std::string& link_name) const;
};

inline ElectricRotorConfig::SharedPtr ElectricPropulsionSystemConfig::getRotor(const std::string& link_name)
{
  return boost::polymorphic_pointer_downcast<ElectricRotorConfig>(rotors.at(link_name));
}

inline ElectricRotorConfig::ConstSharedPtr ElectricPropulsionSystemConfig::getRotor(const std::string& link_name) const
{
  return boost::polymorphic_pointer_downcast<ElectricRotorConfig>(rotors.at(link_name));
}
}  // namespace tobas
