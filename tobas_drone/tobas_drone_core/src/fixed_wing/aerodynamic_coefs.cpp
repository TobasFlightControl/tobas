#include <tobas_yaml_tools/core.hpp>

#include "../../include/tobas_drone_core/fixed_wing/aerodynamic_coefs.hpp"

using namespace std;

namespace tobas
{
bool AerodynamicCoefficients::isValid() const
{
  if (c_lift_0 <= 0)
  {
    cerr << "c_lift_0 must be positive." << endl;
    return false;
  }

  if (c_lift_alpha <= 0)
  {
    cerr << "c_lift_alpha must be positive." << endl;
    return false;
  }

  if (c_drag_0 <= 0)
  {
    cerr << "c_drag_0 must be positive." << endl;
    return false;
  }

  if (c_drag_alpha <= 0)
  {
    cerr << "c_drag_alpha must be positive." << endl;
    return false;
  }

  if (c_side_beta >= 0)
  {
    cerr << "c_side_beta must be negative." << endl;
    return false;
  }

  if (c_roll_beta >= 0)
  {
    cerr << "c_roll_beta must be negative." << endl;
    return false;
  }

  if (c_roll_p >= 0)
  {
    cerr << "c_roll_p must be negative." << endl;
    return false;
  }

  if (c_pitch_alpha >= 0)
  {
    cerr << "c_pitch_alpha must be negative." << endl;
    return false;
  }

  if (c_pitch_q >= 0)
  {
    cerr << "c_pitch_q must be negative." << endl;
    return false;
  }

  if (c_yaw_r >= 0)
  {
    cerr << "c_yaw_r must be negative." << endl;
    return false;
  }

  return true;
}

bool AerodynamicCoefficients::load(const YAML::Node& node)
{
  if (!yaml::load(kCLift0Key, node, c_lift_0))
    return false;

  if (!yaml::load(kCLiftAlphaKey, node, c_lift_alpha))
    return false;

  if (!yaml::load(kCDrag0Key, node, c_drag_0))
    return false;

  if (!yaml::load(kCDragAlphaKey, node, c_drag_alpha))
    return false;

  if (!yaml::load(kCSideBetaKey, node, c_side_beta))
    return false;

  if (!yaml::load(kCRollBetaKey, node, c_roll_beta))
    return false;

  if (!yaml::load(kCRollPKey, node, c_roll_p))
    return false;

  if (!yaml::load(kCRollRKey, node, c_roll_r))
    return false;

  if (!yaml::load(kCPitch0Key, node, c_pitch_0))
    return false;

  if (!yaml::load(kCPitchAlphaKey, node, c_pitch_alpha))
    return false;

  if (!yaml::load(kCPitchAbsBetaKey, node, c_pitch_abs_beta))
    return false;

  if (!yaml::load(kCPitchAlphaRateKey, node, c_pitch_alpha_rate))
    return false;

  if (!yaml::load(kCPitchQKey, node, c_pitch_q))
    return false;

  if (!yaml::load(kCYawBetaKey, node, c_yaw_beta))
    return false;

  if (!yaml::load(kCYawPKey, node, c_yaw_p))
    return false;

  if (!yaml::load(kCYawRKey, node, c_yaw_r))
    return false;

  return true;
}

YAML::Node AerodynamicCoefficients::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kCLift0Key] = c_lift_0;
  node[kCLiftAlphaKey] = c_lift_alpha;
  node[kCDrag0Key] = c_drag_0;
  node[kCDragAlphaKey] = c_drag_alpha;
  node[kCSideBetaKey] = c_side_beta;
  node[kCRollBetaKey] = c_roll_beta;
  node[kCRollPKey] = c_roll_p;
  node[kCRollRKey] = c_roll_r;
  node[kCPitch0Key] = c_pitch_0;
  node[kCPitchAlphaKey] = c_pitch_alpha;
  node[kCPitchAbsBetaKey] = c_pitch_abs_beta;
  node[kCPitchAlphaRateKey] = c_pitch_alpha_rate;
  node[kCPitchQKey] = c_pitch_q;
  node[kCYawBetaKey] = c_yaw_beta;
  node[kCYawPKey] = c_yaw_p;
  node[kCYawRKey] = c_yaw_r;

  return node;
}
}  // namespace tobas
