#include "tobas_drone_core/propulsion_system/ice_propulsion_system/engine.hpp"

#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_math/equation.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_constants/constants.hpp>

using namespace std;

namespace tobas
{
bool EngineConfig::isValid() const
{
  if (torque_const <= 0.) {
    cerr << "Engine torque constant must be positive." << endl;
    return false;
  }

  if (friction_torque <= 0.) {
    cerr << "Engine dynamic friction torque must be positive." << endl;
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
  if (!yaml::load(kTorqueConstantKey, node, torque_const)) {
    return false;
  }

  if (!yaml::load(kDynamicFrictionTorqueKey, node, friction_torque)) {
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

  node[kTorqueConstantKey] = torque_const;
  node[kDynamicFrictionTorqueKey] = friction_torque;
  node[kMaxSpeedKey] = max_speed;
  node[kHardwareIfaceKey] = hw_iface;

  return node;
}

double EngineConfig::computeTorque(double speed, double throttle)
{
  assert(speed >= 0.);
  assert(throttle >= 0.);

  return max(torque_const * g(throttle) * speed - friction_torque, 0.);
}

double EngineConfig::computeSpeed(double throttle, double torque)
{
  assert(throttle >= 0.);
  assert(torque >= 0.);

  return (torque + friction_torque) / (torque_const * g(throttle));
}

double EngineConfig::computeThrottle(double torque, double speed)
{
  assert(torque >= 0.);
  assert(speed >= 0.);

  if (torque == 0.) {
    return tobas::kMinThrot;
  }
  else if (speed == 0.) {
    return tobas::kMaxThrot;
  }

  // スロットル方程式が解けるように定数部分を決める
  const auto throt_const = clamp((torque + friction_torque) / (torque_const * speed), 0., 1.);

  // スロットル方程式を解く
  const auto& [throt_1, throt_2, throt_3] = math::solveCubicEquation(1, -2, 0, throt_const);

  // 定数部分が[0, 1]の範囲にあるとき，解はx <= 0, 0 <= x <= 1, 1 <= xの範囲に1つずつ存在する．
  // そのため3つの解のうち2番目に大きなものを選べばよい．
  double throt_cands[] = { throt_1.real(), throt_2.real(), throt_3.real() };
  sort(begin(throt_cands), end(throt_cands));
  return throt_cands[1];
}

double EngineConfig::g(double throttle)
{
  return math::sqr(throttle) * (2 - throttle);
}

ostream& operator<<(ostream& os, const EngineConfig& arg)
{
  os << "Torque Constant [Nm/(rad/s)]: " << arg.torque_const << endl;
  os << "Dynamic Friction Torque [Nm]: " << arg.friction_torque << endl;
  os << "Maximum Speed [rad/s]: " << arg.max_speed << endl;
  return os;
}
}  // namespace tobas
