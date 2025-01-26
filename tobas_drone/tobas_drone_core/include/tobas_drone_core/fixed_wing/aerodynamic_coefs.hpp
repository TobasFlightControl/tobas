#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
/**
 * @brief Aerodynamics stability derivatives independent of control surfaces.
 * The moment reference point for the wing is at the wing quarter-chord.
 */
class AerodynamicCoefficients
{
  static constexpr char kCLift0Key[] = "c_lift_0";
  static constexpr char kCLiftAlphaKey[] = "c_lift_alpha";
  static constexpr char kCDrag0Key[] = "c_drag_0";
  static constexpr char kCDragAlphaKey[] = "c_drag_alpha";
  static constexpr char kCSideBetaKey[] = "c_side_beta";
  static constexpr char kCRollBetaKey[] = "c_roll_beta";
  static constexpr char kCRollPKey[] = "c_roll_p";
  static constexpr char kCRollRKey[] = "c_roll_r";
  static constexpr char kCPitch0Key[] = "c_pitch_0";
  static constexpr char kCPitchAlphaKey[] = "c_pitch_alpha";
  static constexpr char kCPitchAbsBetaKey[] = "c_pitch_abs_beta";
  static constexpr char kCPitchAlphaRateKey[] = "c_pitch_alpha_rate";
  static constexpr char kCPitchQKey[] = "c_pitch_q";
  static constexpr char kCYawBetaKey[] = "c_yaw_beta";
  static constexpr char kCYawPKey[] = "c_yaw_p";
  static constexpr char kCYawRKey[] = "c_yaw_r";

public:
  // Lift force
  double c_lift_0 = 0;      // [-]
  double c_lift_alpha = 0;  // [/rad]

  // Drag force
  double c_drag_0 = 0;      // [-]
  double c_drag_alpha = 0;  // [/rad]

  // Side force
  double c_side_beta = 0;  // [/rad]

  // Roll moment
  double c_roll_beta = 0;  // [/rad]
  double c_roll_p = 0;     // [s/rad]
  double c_roll_r = 0;     // [s/rad]

  // Pitch moment
  double c_pitch_0 = 0;           // [-]
  double c_pitch_alpha = 0;       // [/rad]
  double c_pitch_abs_beta = 0;    // [/rad]
  double c_pitch_alpha_rate = 0;  // [s/rad]
  double c_pitch_q = 0;           // [s/rad]

  // Yaw moment
  double c_yaw_beta = 0;  // [/rad]
  double c_yaw_p = 0;     // [s/rad]
  double c_yaw_r = 0;     // [s/rad]

  bool isValid() const;
  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};
}  // namespace tobas
