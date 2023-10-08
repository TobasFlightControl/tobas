#pragma once

#include <dh_kdl/treefksolverpos.hpp>

#include "./rotor_axis_extractor.hpp"
#include "./trim_conditions.hpp"

namespace tobas
{
/**
 * @brief トリム状態周りの微小擾乱運動方程式．有次元空力安定微係数を個別に提供する．
 */
class MicroDisturbanceEoM
{
public:
  static constexpr uint32_t kStateSize = 8;
  static constexpr uint32_t kStateIdx_u = 0;
  static constexpr uint32_t kStateIdx_alpha = 1;
  static constexpr uint32_t kStateIdx_beta = 2;
  static constexpr uint32_t kStateIdx_phi = 3;
  static constexpr uint32_t kStateIdx_theta = 4;
  static constexpr uint32_t kStateIdx_p = 5;
  static constexpr uint32_t kStateIdx_q = 6;
  static constexpr uint32_t kStateIdx_r = 7;

  enum ErrorCode
  {
    E_NOERROR = 0,
    E_THRUST_OVERLIMIT = -1,
    E_TRIM_ERROR = -2,
  };

  explicit MicroDisturbanceEoM(const Drone& drone);

  void updateInternalDataStructures();

  /**
   * @brief 内部状態を更新する．
   *
   * @param V 大気に対する機体速度の絶対値 [m/s]
   * @param rho 大気密度 [kg/m^3]
   * @param q 可動関節の角度 [rad]
   *
   * @return ErrorCode Error code
   */
  ErrorCode
  update(const double& V, const double& rho, const double& battery_voltage, const KDL::JntArray& q);

  const ErrorCode& errorCode() const;
  const std::string& errorMessage() const;

  const TrimConditions& trimCondition() const;
  const StabilityDerivativesCG& stabilityDerivativesCG() const;

  const Eigen::Matrix<double, kStateSize, 1>& trimState() const;
  const Eigen::VectorXd& trimInput() const;
  const Eigen::VectorXd& minInput() const;
  const Eigen::VectorXd& maxInput() const;
  Eigen::VectorXd minDeltaInput() const;
  Eigen::VectorXd maxDeltaInput() const;

  /* ピッチ回転のトリムに用いる舵面の添字 */
  const uint32_t& elevatorIndex() const;
  const uint32_t& inputSize() const;

  const Eigen::Matrix<double, kStateSize, kStateSize>& A() const;
  const Eigen::Matrix<double, kStateSize, Eigen::Dynamic>& B() const;

  // X_u (2.2-36)
  const double& u_u() const;
  // X_alpha (2.2-36)
  const double& u_alpha() const;
  const double& u_q() const;
  const double& u_theta() const;

  // Z_u_bar (2.2-37)
  const double& alpha_u() const;
  // Z_alpha_bar (2.2-37)
  const double& alpha_alpha() const;
  const double& alpha_q() const;
  const double& alpha_theta() const;

  // Y_beta_bar (3.2-20)
  const double& beta_beta() const;
  const double& beta_p() const;
  const double& beta_r() const;
  const double& beta_phi() const;

  const double& phi_beta() const;
  const double& phi_p() const;
  const double& phi_r() const;
  const double& phi_phi() const;

  const double& theta_u() const;
  const double& theta_alpha() const;
  const double& theta_q() const;
  const double& theta_theta() const;

  // L_beta_dash (3.2-21)
  const double& p_beta() const;
  // L_p_dash (3.2-21)
  const double& p_p() const;
  // L_r_dash (3.2-21)
  const double& p_r() const;
  const double& p_phi() const;

  // M_u_dash (2.2-39)
  const double& q_u() const;
  // M_alpha_dash (2.2-39)
  const double& q_alpha() const;
  // M_q_dash (2.2-39)
  const double& q_q() const;
  // M_theta_dash (2.2-39)
  const double& q_theta() const;

  // N_beta_dash (3.2-22)
  const double& r_beta() const;
  // N_p_dash (3.2-22)
  const double& r_p() const;
  // N_r_dash (3.2-22)
  const double& r_r() const;
  const double& r_phi() const;

  double u_thrust() const;

  // Z_delta_bar (2.2-37)
  const double& alpha_delta(const uint32_t& cs_idx) const;
  // Y_delta_bar (3.2-20)
  const double& beta_delta(const uint32_t& cs_idx) const;
  // L_delta_bar (3.2-21)
  const double& p_delta(const uint32_t& cs_idx) const;
  // M_delta_bar (2.2-38)
  const double& q_delta(const uint32_t& cs_idx) const;
  // N_delta_bar (3.2.22)
  const double& r_delta(const uint32_t& cs_idx) const;

private:
  ErrorCode error_code_;
  std::string error_msg_;

  const Drone& drone_;

  KDL::ExtTreeFkSolverPos fk_solver_;
  KDL::TreeJntToInertiaSolver inertia_solver_;
  RotorAxisExtractor x_rotors_;
  TrimConditions trim_;

  // 固定値
  double mass_;  // 機体の質量 [kg]
  uint32_t u_size_;
  Eigen::VectorXd min_u_;  // 制御入力の最小値
  Eigen::VectorXd max_u_;  // 制御入力の最大値

  // トリム
  Eigen::Matrix<double, kStateSize, 1> x_0_;  // トリム時の状態
  Eigen::VectorXd u_0_;                       // トリム時の制御入力

  // 各係数のバッファ
  Eigen::Matrix<double, kStateSize, kStateSize> A_;
  Eigen::Matrix<double, kStateSize, Eigen::Dynamic> B_;  // 列数は舵面数と一致

  void setInputLimits(const double& battery_voltage);
};
}  // namespace tobas
