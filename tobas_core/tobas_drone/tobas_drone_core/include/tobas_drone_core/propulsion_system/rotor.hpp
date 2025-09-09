#pragma once

#include <map>

#include "./turning_direction.hpp"

namespace tobas
{
class RotorConfig
{
  static constexpr char kLinkNameKey[] = "link_name";
  static constexpr char kDirectionKey[] = "direction";
  static constexpr char kMomentConstKey[] = "moment_constant";
  static constexpr char kTiltJointName[] = "tilt_joint_name";

public:
  using SharedPtr = std::shared_ptr<RotorConfig>;
  using ConstSharedPtr = std::shared_ptr<const RotorConfig>;

  std::string link_name = "";                          // プロペラのリンク名
  TurningDirection direction = TurningDirection::CCW;  // 回転方向: CCW or CW
  double moment_const = 0.;                            // 反トルク係数 [m]
  std::string tilt_joint_name = "";                    // チルトジョイント名 (空文字の場合は固定軸)

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
