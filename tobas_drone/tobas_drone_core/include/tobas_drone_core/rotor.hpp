#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "./turning_direction.hpp"
#include "./rotor_axis.hpp"

namespace tobas
{
class RotorConfig
{
  static constexpr char kChannelKey[] = "channel";
  static constexpr char kLinkNameKey[] = "link_name";
  static constexpr char kDirectionKey[] = "direction";
  static constexpr char kAxisKey[] = "axis";
  static constexpr char kNumPolesKey[] = "num_poles";
  static constexpr char kMaxRotSpeedKey[] = "max_rot_speed";
  static constexpr char kKvKey[] = "kv";
  static constexpr char kInternalResistanceKey[] = "internal_resistance";
  static constexpr char kPropellerDiameterKey[] = "propeller_diameter";
  static constexpr char kMotorConstKey[] = "motor_constant";
  static constexpr char kMomentConstKey[] = "moment_constant";
  static constexpr char kDragConstKey[] = "drag_constant";
  static constexpr char kRotSpeedCoefKey[] = "rot_speed_coef";

public:
  uint32_t channel = 0;                                      // モータが接続されているチャンネル
  std::string link_name = "";                                // プロペラのリンク名
  turning_direction_t direction = turning_direction_t::CCW;  // 回転方向: CCW or CW
  rotor_axis_t axis = rotor_axis_t::UNKNOWN;                 // 回転軸
  uint32_t num_poles = 0;                                    // モータの極数
  double kv = 0;                                             // モータのKV値 [rad/s/V]
  double internal_resistance;                                // モータの内部抵抗 [Ω]
  double propeller_diameter;                                 // プロペラの直径 [m]
  double max_rot_speed = 0;                                  // 最大連続回転数 [rad/s]
  double motor_constant = 0;                                 // 推力係数 [kg*m/rad^2]
  double moment_constant = 0;                                // 反トルク係数 [m]
  double drag_constant = 0;                                  // 空気効力定数 [kg/rad]

  bool isValid() const;
  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  /* CCW = 1, CW = -1 */
  inline int sign() const
  {
    return tobas::sign(direction);
  }
};

using RotorConfigs = std::vector<RotorConfig>;
}  // namespace tobas
