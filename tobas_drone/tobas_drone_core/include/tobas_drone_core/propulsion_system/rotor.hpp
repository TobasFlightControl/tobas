#pragma once

#include <map>

#include "./turning_direction.hpp"
#include "./rotor_axis.hpp"

namespace tobas
{

class RotorConfig
{
  static constexpr char kLinkNameKey[] = "link_name";
  static constexpr char kDirectionKey[] = "direction";
  static constexpr char kAxisKey[] = "axis";
  static constexpr char kMomentConstKey[] = "moment_constant";
  static constexpr char kTiltJointName[] = "tilt_joint_name";

public:
  using SharedPtr = std::shared_ptr<RotorConfig>;
  using ConstSharedPtr = std::shared_ptr<const RotorConfig>;

  std::string link_name = "";                                // プロペラのリンク名
  turning_direction_t direction = turning_direction_t::CCW;  // 回転方向: CCW or CW
  rotor_axis_t axis = rotor_axis_t::UNKNOWN;                 // 回転軸
  double moment_const = 0.;                                  // 反トルク係数 [m]
  std::string tilt_joint_name = "";  // ティルトジョイント名 (空文字の場合は固定軸)

  virtual bool isValid() const;

  virtual bool load(const YAML::Node& node);
  virtual YAML::Node dump() const;

  /* CCW = 1, CW = -1 */
  inline int sign() const;
};

using RotorConfigMap = std::map<std::string, RotorConfig::SharedPtr>;  // Link Name -> RotorConfig

inline int RotorConfig::sign() const
{
  return tobas::sign(direction);
}
}  // namespace tobas
