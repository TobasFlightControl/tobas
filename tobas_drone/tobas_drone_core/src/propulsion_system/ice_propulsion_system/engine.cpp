#include <cassert>

#include <tobas_math/equation.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/engine.hpp"

using namespace std;

namespace tobas
{
bool EngineConfig::isValid() const
{
  if (torque_const <= 0.)
  {
    cerr << "Engine torque constant must be positive." << endl;
    return false;
  }

  if (friction_torque <= 0.)
  {
    cerr << "Engine dynamic friction torque must be positive." << endl;
    return false;
  }

  return true;
}

bool EngineConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kTorqueConstantKey, node, torque_const))
    return false;

  if (!yaml::load(kDynamicFrictionTorqueKey, node, friction_torque))
    return false;

  return true;
}

YAML::Node EngineConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTorqueConstantKey] = torque_const;
  node[kDynamicFrictionTorqueKey] = friction_torque;

  return node;
}

double EngineConfig::solveThrottleEquation(double torque, double speed)
{
  assert(torque >= 0.);
  assert(speed >= 0.);

  if (torque == 0.)
    return tobas::kMinThrot;
  else if (speed == 0.)
    return tobas::kMaxThrot;

  // スロットル方程式が解けるように定数部分を決める
  const auto throt_const = clamp((torque + friction_torque) / (torque_const * speed), 0., 1.);

  // スロットル方程式を解く
  const auto& [throt_1, throt_2, throt_3] = math::solveCubicEquation(1, -2, 0, throt_const);
  const array<double, 3> throt_cands(throt_1.real(), throt_2.real(), throt_3.real());  // スロットル候補

  // 定数部分が[0, 1]の範囲にあるとき，[0, 1]を満たすスロットル解は1つだけ
  for (const auto& throt : throt_cands)
    if (kMinThrot <= throt && throt <= kMaxThrot)
      return throt;
}
}  // namespace tobas
