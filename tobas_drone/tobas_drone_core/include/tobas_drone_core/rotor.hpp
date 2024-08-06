#pragma once

#include <cinttypes>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "./turning_direction.hpp"
#include "./rotor_axis.hpp"
#include "./esc.hpp"

namespace tobas
{
class RotorConfig
{
  static constexpr char kChannelKey[] = "channel";
  static constexpr char kLinkNameKey[] = "link_name";
  static constexpr char kDirectionKey[] = "direction";
  static constexpr char kAxisKey[] = "axis";
  static constexpr char kEscModeKey[] = "esc_mode";
  static constexpr char kNumPolesKey[] = "num_poles";
  static constexpr char kMaxRotSpeedKey[] = "max_rot_speed";
  static constexpr char kMotorConstKey[] = "motor_constant";
  static constexpr char kMomentConstKey[] = "moment_constant";
  static constexpr char kRotSpeedCoefKey[] = "rot_speed_coef";

public:
  uint32_t channel = 0;                                      // モータが接続されているチャンネル
  std::string link_name = "";                                // プロペラのリンク名
  turning_direction_t direction = turning_direction_t::CCW;  // 回転方向: CCW or CW
  rotor_axis_t axis = rotor_axis_t::UNKNOWN;                 // 回転軸
  esc_mode_t esc_mode = esc_mode_t::BLHELI_OPEN_LOOP;        // ESCのスロットルの解釈方式
  uint32_t num_poles = 0;                                    // モータの極数
  double max_rot_speed = 0;                                  // 最大連続回転数 [rad/s]
  double motor_constant = 0;                                 // 推力係数 [kg*m/rad^2]
  double moment_constant = 0;                                // 反トルク係数 [m]
  double drag_constant = 0;                                  // 空気効力定数 [kg/rad]
  std::pair<double, double> rot_speed_coefs = { 0, 0 };      // V = c1 w + c2 w^2 (V[V], w[rad/s])

  bool isValid() const;
  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  /* CCW = 1, CW = -1 */
  inline int sign() const
  {
    switch (direction)
    {
      case CCW:
        return 1;
      case CW:
        return -1;
      default:
        throw;
    }
  }
};

using RotorConfigs = std::vector<RotorConfig>;
}  // namespace tobas
