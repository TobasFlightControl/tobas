#include <iostream>

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp"

using namespace std;

namespace tobas
{
bool ICEPropulsionSystemConfig::isValid() const
{
  // Rotors
  for (const auto& [_, rotor] : rotors)
  {
    if (!rotor->isValid())
    {
      cerr << "The configurations of rotor \"" << rotor->link_name << "\" are invalid." << endl;
      return false;
    }
  }

  // Engine
  if (!engine.isValid())
    return false;

  return true;
}

bool ICEPropulsionSystemConfig::load(const YAML::Node& node)
{
  // Rotors
  rotors.clear();
  if (!node[kRotorsKey].IsSequence())
  {
    cerr << "Rotors field is not defined." << endl;
    return false;
  }
  for (const auto& rotor_node : node[kRotorsKey])
  {
    const auto rotor = make_shared<ICERotorConfig>();
    if (!rotor->load(rotor_node))
    {
      cerr << "Failed to load the configurations of rotors." << endl;
      return false;
    }
    rotors[rotor->link_name] = rotor;
  }

  // Engine
  if (!node[kEngineKey].IsDefined())
  {
    cerr << "Engine field is not defined." << endl;
    return false;
  }
  if (!engine.load(node[kEngineKey]))
  {
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
  for (const auto& [_, rotor] : rotors)
    node[kRotorsKey].push_back(rotor->dump());

  // Engine
  node[kEngineKey] = engine.dump();

  return node;
}

propulsion_system_t ICEPropulsionSystemConfig::type() const
{
  return propulsion_system_t::ICE;
}

double ICEPropulsionSystemConfig::minSpeed(const std::string& link_name) const
{
  // TODO: 静止摩擦を考慮して最小回転数を決める
  (void)link_name;
  return 0.;
}

double ICEPropulsionSystemConfig::maxSpeed(const std::string& link_name) const
{
  // 全てのプロペラの空気抵抗が最小のときにエンジン回転数を計算 (memo: 3-26)
  double sum = 0.;
  for (const auto& [_, rotor] : rotors)
  {
    const auto irotor = dynamic_pointer_cast<ICERotorConfig>(rotor);
    sum += irotor->minMotorConst() * irotor->moment_const / math::sqr(irotor->gear_ratio);
  }
  const auto engine_speed =
    (engine.torque_const + sqrt(math::sqr(engine.torque_const) - 4 * sum * engine.friction_torque)) / (2 * sum);

  // エンジン回転数からプロペラ回転数を計算
  const auto irotor = getRotor(link_name);
  return irotor->speedEngineToRotor(engine_speed);
}

double ICEPropulsionSystemConfig::minThrust(const std::string& link_name) const
{
  // TODO: 最小回転数のときの推力
  (void)link_name;
  return 0.;
}

double ICEPropulsionSystemConfig::maxThrust(const std::string& link_name) const
{
  // 全てのプロペラの空気抵抗が最大のときのエンジン回転数を計算
  double sum = 0.;
  for (const auto& [_, rotor] : rotors)
  {
    const auto irotor = dynamic_pointer_cast<ICERotorConfig>(rotor);
    sum += irotor->maxMotorConst() * irotor->moment_const / math::sqr(irotor->gear_ratio);
  }
  const auto engine_speed =
    (engine.torque_const + sqrt(math::sqr(engine.torque_const) - 4 * sum * engine.friction_torque)) / (2 * sum);

  // エンジン回転数から推力を計算
  // これは最大推力の最小値であり，ピッチ角が小さい時はより大きくできる (memo: 3-26)
  const auto irotor = getRotor(link_name);
  return irotor->thrustFromPitch(engine_speed, irotor->pitch_range.upper);
}
}  // namespace tobas
