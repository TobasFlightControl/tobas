#pragma once

#include <dh_linear_control/state_spaces.hpp>
#include <dh_kdl/treekdlmodel.hpp>

#include <tobas_tools/drone.hpp>

namespace tobas_fixed_wing_controller
{
/**
 * @brief 固定翼機の微小擾乱状態方程式．
 * cf. https://www.morikita.co.jp/books/mid/069081
 */
class FixedWingMicroDisturbanceDynamics : public ctrl::LinearDynamics
{
public:
  static constexpr int stateSize = 8;
  static constexpr int stateIdx_u = 0;
  static constexpr int stateIdx_alpha = 1;
  static constexpr int stateIdx_beta = 2;
  static constexpr int stateIdx_phi = 3;
  static constexpr int stateIdx_theta = 4;
  static constexpr int stateIdx_p = 5;
  static constexpr int stateIdx_q = 6;
  static constexpr int stateIdx_r = 7;

  using StateVector = Eigen::Matrix<double, stateSize, 1>;

  explicit FixedWingMicroDisturbanceDynamics(const Drone& drone);

  void update(double V);

  const Eigen::Matrix<double, stateSize, 1>& trimState() const;
  const Eigen::VectorXd& trimInput() const;

  double trimState_u() const;
  double trimState_alpha() const;
  double trimState_beta() const;
  double trimState_phi() const;
  double trimState_theta() const;
  double trimState_p() const;
  double trimState_q() const;
  double trimState_r() const;

private:
  const Drone& drone_;
  KDL::TreeKDLModel kdl_model_;

  Eigen::Matrix<double, stateSize, 1> trim_state_;  // トリム時の状態
  Eigen::VectorXd trim_input_;                      // トリム時の制御入力
};
}  // namespace tobas_fixed_wing_controller
