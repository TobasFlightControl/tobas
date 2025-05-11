#include "tobas_drone_core/propulsion_system/ice_propulsion_system/engine.hpp"

#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_yaml_tools/core.hpp>

using namespace std;

namespace tobas
{
bool EngineConfig::isValid() const
{
  if (engine_const.first <= 0. || engine_const.second <= 0.) {
    cerr << "Engine constants must be positive." << endl;
    return false;
  }

  if (max_speed <= 0.) {
    cerr << "Engine maximum speed must be positive." << endl;
    return false;
  }

  return true;
}

bool EngineConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kEngineConstantKey, node, engine_const)) {
    return false;
  }

  if (!yaml::load(kMaxSpeedKey, node, max_speed)) {
    return false;
  }

  if (!yaml::load(kHardwareIfaceKey, node, hw_iface)) {
    return false;
  }

  return true;
}

YAML::Node EngineConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kEngineConstantKey] = engine_const;
  node[kMaxSpeedKey] = max_speed;
  node[kHardwareIfaceKey] = hw_iface;

  return node;
}

double EngineConfig::computeTorque(double speed, double throttle)
{
  assert(speed >= 0.);
  assert(0. <= throttle && throttle <= 1.);

  if (throttle == 0.) {
    return 0.;
  }

  const auto& [A, B] = engine_const;

  // FIXME: 実際はゼロスロットル (アイドリング) でも出力トルクはゼロではなく，エンジンモデルの改善が必要．
  const auto phi = M_PI_2 * throttle;
  const auto f = math::sqr(A / (1 - cos(phi)));
  return 2 * B * speed / (sqrt(1 + 4 * B * math::sqr(speed) * f) + 1);
}

double EngineConfig::computeThrottle(double torque, double speed)
{
  assert(torque >= 0.);
  assert(speed >= 0.);

  if (speed == 0.) {
    return 0.;
  }

  const auto& [A, B] = engine_const;

  const auto cos_phi = 1. - A * torque / sqrt(B - torque / speed);
  const auto phi = acos(clamp(cos_phi, 0., 1.));
  return phi / M_PI_2;
}

ostream& operator<<(ostream& os, const EngineConfig& arg)
{
  os << "Engine Constant: " << arg.engine_const.first << ", " << arg.engine_const.second << endl;
  os << "Maximum Speed [rad/s]: " << arg.max_speed << endl;
  return os;
}
}  // namespace tobas
