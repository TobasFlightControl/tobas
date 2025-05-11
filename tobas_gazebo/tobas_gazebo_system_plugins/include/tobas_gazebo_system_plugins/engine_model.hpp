#pragma once

#include <tobas_nlp/newton_1d.hpp>

#include "./filter/asymmetric_first_order_filter.hpp"
#include "./ice_rotor_model.hpp"

namespace gazebo
{
class EngineModel
{
  using self = EngineModel;

public:
  explicit EngineModel(const ICERotorModelMap& rotors);

  bool initialize(const sdf::ElementConstPtr& sdf);

  /* 回転数 [rad/s] */
  double getSpeed() const;

  /* 回転位置 [rad] */
  double getPosition() const;

  void setThrottle(const double& throttle);

  bool step(const double& dt);

private:
  const ICERotorModelMap& rotors_;

  // SDF parameters
  std::pair<double, double> engine_const_;  // A, B (memo: 3-28)
  double max_speed_;                        // [rad/s]
  double time_const_up_;                    // [s]
  double time_const_down_;                  // [s]

  // Command
  double throttle_ = 0.;  // スロットル開度 [0, 1]

  // State
  double position_ = 0.;  // 位置 [rad]
  AsymmetricFirstOrderFilter<double> speed_filter_;

  // Solver
  nlp::NewtonSolver1d newton_;

  bool getSdfParams(const sdf::ElementConstPtr& sdf);

  /* エンジンスロットルとティルト角から定常回転数を求める (memo: 3-29) */
  double computeSteadySpeed();

  /* ニュートン法ソルバーに渡す関数 (memo: 3-29) */
  double speedFunc(double omega) const;
  double speedFuncDeriv(double omega) const;

  double calc_phi() const;
  double calc_f() const;
  double calc_k() const;
};
}  // namespace gazebo
