#include "tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp"

#include <iostream>

#include <tobas_nlp/newton_1d.hpp>

using namespace std;

namespace tobas
{
bool ICEPropulsionSystemConfig::isValid() const
{
  // Rotors
  for (const auto& [_, rotor] : rotors) {
    if (!rotor->isValid()) {
      cerr << "The configurations of rotor \"" << rotor->link_name << "\" are invalid." << endl;
      return false;
    }
  }

  // Engine
  if (!engine.isValid()) {
    return false;
  }

  return true;
}

bool ICEPropulsionSystemConfig::load(const YAML::Node& node)
{
  // Rotors
  rotors.clear();
  if (!node[kRotorsKey].IsSequence()) {
    cerr << "Rotors field is not defined." << endl;
    return false;
  }
  for (const auto& rotor_node : node[kRotorsKey]) {
    const auto rotor = make_shared<ICERotorConfig>();
    if (!rotor->load(rotor_node)) {
      cerr << "Failed to load the configurations of rotors." << endl;
      return false;
    }
    rotors[rotor->link_name] = rotor;
  }

  // Engine
  if (!node[kEngineKey].IsDefined()) {
    cerr << "Engine field is not defined." << endl;
    return false;
  }
  if (!engine.load(node[kEngineKey])) {
    cerr << "Failed to load the configurations of engine." << endl;
    return false;
  }

  return true;
}

YAML::Node ICEPropulsionSystemConfig::dump() const
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

propulsion_system_t ICEPropulsionSystemConfig::type() const
{
  return propulsion_system_t::ICE;
}

double ICEPropulsionSystemConfig::minSpeed(const string&) const
{
  return 0.;
}

double ICEPropulsionSystemConfig::maxSpeed(const string& link_name) const
{
  return engine.max_speed / getRotor(link_name)->gear_ratio;
}

double ICEPropulsionSystemConfig::minThrust(const string&) const
{
  return 0.;
}

double ICEPropulsionSystemConfig::maxThrust(const string& link_name) const
{
  const auto rotor = getRotor(link_name);
  const auto max_motor_const = rotor->motorConst(rotor->pitch_limit.upper);
  const auto max_speed = engine.max_speed / rotor->gear_ratio;
  return max_motor_const * math::sqr(max_speed);
}

double ICEPropulsionSystemConfig::thrustFromThrottle(const std::string& link_name, double throttle) const
{
  const auto rotor = getRotor(link_name);
  const auto engine_speed = computeEngineSpeed(throttle);
  return rotor->thrustFromPitch(engine_speed, rotor->pitch_ref);  // XXX: 参照ピッチ角のときの推力を返す
}

double ICEPropulsionSystemConfig::computeEngineSpeed(double throttle) const
{
  // FIXME: 実際はゼロスロットルでもトルクは発生する (アイドリング)
  if (throttle <= std::numeric_limits<double>::epsilon()) {
    return 0.;
  }

  nlp::NewtonSolver1d newton;

  newton.initialize(
    bind(&self::speedFunc, this, throttle, std::placeholders::_1),
    bind(&self::speedFuncDeriv, this, throttle, std::placeholders::_1));

  double engine_speed = engine.max_speed;
  if (newton.solve(engine_speed) < 0) {
    if (newton.solve(engine_speed) < 0) {
      cerr << "Failed to solve engine dynamics equation: " << newton.errorMessage() << endl;
      return 0.;
    }
  }

  return engine_speed;
}

double ICEPropulsionSystemConfig::speedFunc(double throttle, double omega) const
{
  const auto& B = engine.engine_const.second;
  const auto f = calc_f(throttle);
  const auto k = calc_k();
  return f * math::sqr(k) * math::quat(omega) + k * omega - B;
}

double ICEPropulsionSystemConfig::speedFuncDeriv(double throttle, double omega) const
{
  const auto f = calc_f(throttle);
  const auto k = calc_k();
  return 4 * f * math::sqr(k) * math::cube(omega) + k;
}

double ICEPropulsionSystemConfig::calc_phi(double throttle) const
{
  return M_PI_2 * throttle;
}

double ICEPropulsionSystemConfig::calc_f(double throttle) const
{
  const auto& A = engine.engine_const.first;
  const auto phi = calc_phi(throttle);
  return math::sqr(A / (1 - cos(phi)));
}

double ICEPropulsionSystemConfig::calc_k() const
{
  double res = 0.;
  for (const auto& [_, rotor] : rotors) {
    const auto irotor = boost::polymorphic_pointer_downcast<tobas::ICERotorConfig>(rotor);
    res += irotor->motorConst(irotor->pitch_ref) * irotor->moment_const / math::cube(irotor->gear_ratio);
  }
  return res;
}
}  // namespace tobas
