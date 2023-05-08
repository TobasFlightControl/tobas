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
  uint32_t inputSize() const;

private:
  const Drone& drone_;
  KDL::TreeKDLModel kdl_model_;

  // 固定
  std::vector<uint32_t> hor_prop_idxes_;     // X軸正方向を向いたプロペラの添字
  Eigen::VectorXd min_u_;                    // 制御入力の最小値 (固定)
  Eigen::VectorXd max_u_;                    // 制御入力の最大値 (固定)

  Eigen::Matrix<double, stateSize, 1> x_0_;  // トリム時の状態
  Eigen::VectorXd u_0_;                      // トリム時の制御入力

  void setInputLimits();
};
}  // namespace tobas_fixed_wing_controller
