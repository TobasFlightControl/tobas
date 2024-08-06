#pragma once

#include <cinttypes>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <tobas_std_tools/range.hpp>

namespace tobas
{
/**
 * @brief Control sufrace.
 * The moment reference point for the wing is at the wing quarter-chord.
 * A rotation axis parallel to the Y or Z axis is assumed.
 */
class ControlSurface
{
  static constexpr char kChannelKey[] = "channel";
  static constexpr char kJointNameKey[] = "joint_name";
  static constexpr char kAngleLimitKey[] = "angle_limit";
  static constexpr char kMaxAngleRateMKey[] = "max_angle_rate";
  static constexpr char kCLiftDeltaKey[] = "c_lift_delta";
  static constexpr char kCDragAbsDeltaKey[] = "c_drag_abs_delta";
  static constexpr char kCSideDeltaKey[] = "c_side_delta";
  static constexpr char kCRollDeltaKey[] = "c_roll_delta";
  static constexpr char kCPitchDeltaKey[] = "c_pitch_delta";
  static constexpr char kCYawDeltaKey[] = "c_yaw_delta";

public:
  uint32_t channel = 0;  // モータが接続されているチャンネル
  std::string joint_name = "";
  tobas_std::Range<double> angle_limit = { 0, 0 };  // [rad]
  double max_angle_rate = 0;                        // [rad/s]

  double c_lift_delta = 0;      // [/rad]
  double c_drag_abs_delta = 0;  // [/rad], 舵角の正負にかかわらず抗力が発生するモデル
  double c_side_delta = 0;      // [/rad]
  double c_roll_delta = 0;      // [/rad]
  double c_pitch_delta = 0;     // [/rad]
  double c_yaw_delta = 0;       // [/rad]

  bool isValid() const;
  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};

using ControlSurfaces = std::vector<ControlSurface>;
}  // namespace tobas
