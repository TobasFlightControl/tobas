#pragma once

#include "../propulsion_system.hpp"
#include "./engine.hpp"
#include "./ice_rotor.hpp"

namespace tobas
{
class ICEPropulsionSystemConfig : public PropulsionSystemConfig
{
  static constexpr char kEngineKey[] = "engine";

public:
  using SharedPtr = std::shared_ptr<ICEPropulsionSystemConfig>;
  using ConstSharedPtr = std::shared_ptr<const ICEPropulsionSystemConfig>;

  EngineConfig engine;

  bool isValid() const override;

  bool load(const YAML::Node& node) override;
  YAML::Node dump() const override;

  propulsion_system_t type() const override;

  double minSpeed(const std::string& link_name) const override;
  double maxSpeed(const std::string& link_name) const override;
  double minThrust(const std::string& link_name) const override;
  double maxThrust(const std::string& link_name) const override;

  inline ICERotorConfig::SharedPtr getRotor(const std::string& link_name);
  inline ICERotorConfig::ConstSharedPtr getRotor(const std::string& link_name) const;
};

inline ICERotorConfig::SharedPtr ICEPropulsionSystemConfig::getRotor(const std::string& link_name)
{
  return static_pointer_cast<ICERotorConfig>(rotors.at(link_name));
}

inline ICERotorConfig::ConstSharedPtr ICEPropulsionSystemConfig::getRotor(const std::string& link_name) const
{
  return static_pointer_cast<ICERotorConfig>(rotors.at(link_name));
}
}  // namespace tobas
