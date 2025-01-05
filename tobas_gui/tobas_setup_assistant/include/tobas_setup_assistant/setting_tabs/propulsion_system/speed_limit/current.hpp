#pragma once

#include "./base.hpp"
#include "../motor.hpp"
#include "../aerodynamics/aerodynamics.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion
{
/**
 * @brief 最大連続電流からロータの最大回転数を推定．
 * 最大連続電流を超える高負荷になると，T = kt Iが成り立たなくなり，トルクが飽和する．
 * また，モータが加熱することによりコイルのインダクタンスが増加することも回転数低下の原因となる．
 * cf. [ブラシレスモータ効率の良い回し方](https://www.cqpub.co.jp/hanbai/books/MTR/MTRZ201310/MTRZ201310.pdf)
 */
class SpeedLimitWidget_Current : public SpeedLimitWidget_Base
{
  Q_OBJECT

public:
  explicit SpeedLimitWidget_Current(MotorWidget* motor, AerodynamicsWidget* aerodynamics);

  const char* name() const override;

  void onInit() override;

  bool isValid() override;

  double maxRotSpeed() const override;

private:
  MotorWidget* motor_;
  AerodynamicsWidget* aerodynamics_;
};
}  // namespace propulsion
}  // namespace setup_assistant
}  // namespace gui
