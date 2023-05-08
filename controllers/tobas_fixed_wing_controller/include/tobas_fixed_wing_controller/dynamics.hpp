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
  static constexpr int kStateSize = 8;
  static constexpr int kStateIdx_u = 0;
  static constexpr int kStateIdx_alpha = 1;
  static constexpr int kStateIdx_beta = 2;
  static constexpr int kStateIdx_phi = 3;
  static constexpr int kStateIdx_theta = 4;
  static constexpr int kStateIdx_p = 5;
  static constexpr int kStateIdx_q = 6;
  static constexpr int kStateIdx_r = 7;

  using StateVector = Eigen::Matrix<double, kStateSize, 1>;

  explicit FixedWingMicroDisturbanceDynamics(const Drone& drone);

  /**
   * @brief 連続時間状態方程式を更新する．
   *
   * @param V 機体速度の絶対値 [m/s]
   * @param altitude 高度 [m]．大気密度の推定に使用．
   */
  void update(double V, double altitude);

  const Eigen::Matrix<double, kStateSize, 1>& trimState() const;
  const Eigen::VectorXd& trimInput() const;

  const Eigen::VectorXd& minInput() const;
  const Eigen::VectorXd& maxInput() const;
  Eigen::VectorXd minDeltaInput() const;
  Eigen::VectorXd maxDeltaInput() const;

  double trimState_u() const;
  double trimState_alpha() const;
  double trimState_beta() const;
  double trimState_phi() const;
  double trimState_theta() const;
  double trimState_p() const;
  double trimState_q() const;
  double trimState_r() const;

  uint32_t horizontalPropIndex(uint32_t input_index) const;
  uint32_t horizontalPropsSize() const;
  uint32_t controlSurfacesSize() const;

private:
  const Drone& drone_;
  KDL::TreeKDLModel kdl_model_;

  // 固定値
  std::vector<uint32_t> hor_prop_idxes_;  // X軸正方向を向いたプロペラの添字
  Eigen::VectorXd min_u_;                 // 制御入力の最小値 (固定)
  Eigen::VectorXd max_u_;                 // 制御入力の最大値 (固定)
  double mass_;                           // 機体の質量 [kg]

  Eigen::Matrix<double, kStateSize, 1> x_0_;  // トリム時の状態
  Eigen::VectorXd u_0_;                       // トリム時の制御入力

  void setInputLimits();
  void updateTrimStateInput(double V, double rho);
  void updateA(double V, double rho);
  void updateB(double V, double rho);
};
}  // namespace tobas_fixed_wing_controller
