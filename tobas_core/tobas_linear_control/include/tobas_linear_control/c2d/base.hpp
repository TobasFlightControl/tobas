#pragma once

#include "../state_spaces.hpp"

namespace ctrl
{
/**
 * @brief 連続時間状態方程式を離散時間状態方程式に変換するクラスの基底．
 */
class BaseC2D
{
public:
  virtual LinearDynamics convert(const LinearDynamics& cont, const double& dt) = 0;
};
}  // namespace ctrl
