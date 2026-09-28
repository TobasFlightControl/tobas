// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp"

#include <iostream>
#include <memory>
#include <ranges>

#include <tobas_constants/throttle.hpp>
#include <tobas_nlp/newton_1d.hpp>

namespace tobas
{
namespace
{
constexpr char kEngineKey[] = "engine";
constexpr char kRotorsKey[] = "rotors";
}  // namespace

std::expected<void, std::string> IcePropulsionSystemConfig::validate() const
{
  for (const auto& [link_name, rotor] : rotors) {
    if (!rotor) {
      return std::unexpected("Rotor '" + link_name + "': configuration is missing.");
    }
    if (const auto result = rotor->validate(); !result) {
      return std::unexpected("Rotor '" + link_name + "': " + result.error());
    }
  }

  return engine.validate();
}

std::expected<void, std::string> IcePropulsionSystemConfig::load(const YAML::Node& root_node)
{
  if (!root_node.IsDefined() || !root_node.IsMap()) {
    return std::unexpected("ICE propulsion system node must be a map.");
  }

  clear();

  // Rotors
  const auto rotors_node = root_node[kRotorsKey];
  if (!rotors_node.IsDefined()) {
    return std::unexpected(std::string("'") + kRotorsKey + "' is not defined.");
  }
  if (!rotors_node.IsSequence()) {
    return std::unexpected(std::string("'") + kRotorsKey + "' must be a sequence.");
  }
  for (const auto& [idx, rotor_node] : std::views::enumerate(rotors_node)) {
    const auto irotor = std::make_shared<IceRotorConfig>();
    if (const auto result = irotor->load(rotor_node); !result) {
      return std::unexpected("Rotors[" + std::to_string(idx) + "]: " + result.error());
    }
    rotors[irotor->link_name] = irotor;
  }

  // Engine
  const auto engine_node = root_node[kEngineKey];
  if (!engine_node.IsDefined()) {
    return std::unexpected(std::string("'") + kEngineKey + "' is not defined.");
  }
  if (const auto result = engine.load(engine_node); !result) {
    return std::unexpected("Engine: " + result.error());
  }

  // Maximum engine speed
  max_engine_speed_ = computeEngineSpeed(kMaxThrot);

  return {};
}

YAML::Node IcePropulsionSystemConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  // Rotors
  node[kRotorsKey] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& [_, rotor] : rotors) {
    node[kRotorsKey].push_back(rotor->dump());
  }

  // Engine
  node[kEngineKey] = engine.dump();

  return node;
}

PropulsionSystem IcePropulsionSystemConfig::type() const
{
  return PropulsionSystem::kIce;
}

double IcePropulsionSystemConfig::minSpeed(const std::string&)
{
  return 0.0;
}

double IcePropulsionSystemConfig::maxSpeed(const std::string& link_name)
{
  return maxEngineSpeed() / getRotor(link_name)->gear_ratio;
}

double IcePropulsionSystemConfig::minThrust(const std::string&)
{
  return 0.0;
}

double IcePropulsionSystemConfig::maxThrust(const std::string& link_name)
{
  const auto irotor = getRotor(link_name);
  const auto max_motor_const = irotor->motorConst(irotor->pitch_limit.upper);
  const auto max_speed = maxEngineSpeed() / irotor->gear_ratio;
  return max_motor_const * math::sqr(max_speed);
}

double IcePropulsionSystemConfig::thrustFromThrottle(const std::string& link_name, double throttle)
{
  const auto irotor = getRotor(link_name);
  const auto engine_speed = computeEngineSpeed(throttle);
  return irotor->thrustFromPitch(engine_speed, irotor->center_pitch);  // Return thrust at the center pitch angle.
}

double IcePropulsionSystemConfig::maxEngineSpeed()
{
  // Compute the engine speed at full throttle only once.
  if (!max_engine_speed_) {
    max_engine_speed_ = computeEngineSpeed(kMaxThrot);
  }

  return *max_engine_speed_;
}

double IcePropulsionSystemConfig::computeEngineSpeed(double throttle) const
{
  // FIXME: Torque is actually generated even at zero throttle due to idling.
  if (throttle <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  nlp::NewtonSolver1d newton;

  newton.initialize(
    bind(&self::speedFunc, this, throttle, std::placeholders::_1),
    bind(&self::speedFuncDeriv, this, throttle, std::placeholders::_1));

  double engine_speed = 0.0;
  if (newton.solve(engine_speed) < 0) {
    std::cerr << "Failed to solve the engine dynamics equation: " << newton.errorMessage() << std::endl;
    return 0.0;
  }

  return engine_speed;
}

double IcePropulsionSystemConfig::speedFunc(double throttle, double omega) const
{
  const auto& B = engine.engine_const.second;
  const auto f = calc_f(throttle);
  const auto k = calc_k();
  return f * math::sqr(k) * math::quar(omega) + k * omega - B;
}

double IcePropulsionSystemConfig::speedFuncDeriv(double throttle, double omega) const
{
  const auto f = calc_f(throttle);
  const auto k = calc_k();
  return 4 * f * math::sqr(k) * math::cube(omega) + k;
}

double IcePropulsionSystemConfig::calc_phi(double throttle) const
{
  return M_PI_2 * throttle;
}

double IcePropulsionSystemConfig::calc_f(double throttle) const
{
  const auto& A = engine.engine_const.first;
  const auto phi = calc_phi(throttle);
  return math::sqr(A / (1 - std::cos(phi)));
}

double IcePropulsionSystemConfig::calc_k() const
{
  double res = 0.0;
  for (const auto& [_, rotor] : rotors) {
    const auto irotor = std::static_pointer_cast<IceRotorConfig>(rotor);
    const auto& phi_r = irotor->center_pitch;
    const auto& n = irotor->gear_ratio;
    res += irotor->motorConst(phi_r) * irotor->momentConst(phi_r) / math::cube(n);
  }
  return res;
}

IceRotorConfig::SharedPtr IcePropulsionSystemConfig::getRotor(const std::string& link_name)
{
  const auto it = rotors.find(link_name);
  if (it == rotors.end()) {
    std::cerr << "ICE rotor link '" << link_name << "' is not found." << std::endl;
    return nullptr;
  }
  return std::static_pointer_cast<IceRotorConfig>(it->second);
}

IceRotorConfig::ConstSharedPtr IcePropulsionSystemConfig::getRotor(const std::string& link_name) const
{
  const auto it = rotors.find(link_name);
  if (it == rotors.end()) {
    std::cerr << "ICE rotor link '" << link_name << "' is not found." << std::endl;
    return nullptr;
  }
  return std::static_pointer_cast<IceRotorConfig>(it->second);
}
}  // namespace tobas
