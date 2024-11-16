#pragma once

#include <tobas_kdl/treefksolverpos.hpp>

#include "./solveri.hpp"
#include "./rotor_axis_extractor.hpp"
#include "./fw_trim_conditions.hpp"

namespace tobas
{
/**
 * @brief トリム状態周りの微小擾乱運動方程式．有次元空力安定微係数を個別に提供する．
 */
class MicroDisturbanceEoM : public SolverI
{
public:
  static constexpr size_t kStateIdx_u = 0;
  static constexpr size_t kStateIdx_alpha = 1;
  static constexpr size_t kStateIdx_beta = 2;
  static constexpr size_t kStateIdx_phi = 3;
  static constexpr size_t kStateIdx_theta = 4;
  static constexpr size_t kStateIdx_p = 5;
  static constexpr size_t kStateIdx_q = 6;
  static constexpr size_t kStateIdx_r = 7;
  static constexpr size_t kStateSize = 8;

  using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;
  using StateVector = Eigen::Matrix<double, kStateSize, 1>;

  explicit MicroDisturbanceEoM(const Drone& drone, const kdl::Tree& tree);

  void updateInternalDataStructures() override;

  /**
   * @brief 内部状態を更新する．
   *
   * @param V 大気に対する機体速度の絶対値 [m/s]
   * @param rho 大気密度 [kg/m^3]
   * @param q 可動関節の角度 [rad]
   */
  int update(const double& V, const double& rho, const double& battery_voltage, const kdl::JntArray& q);

  inline const TrimConditions& trimCondition() const;
  inline const StabilityDerivativesCG& stabilityDerivativesCG() const;

  inline const Eigen::Matrix<double, kStateSize, 1>& trimState() const;
  inline const Eigen::VectorXd& trimInput() const;
  inline const Eigen::VectorXd& minInput() const;
  inline const Eigen::VectorXd& maxInput() const;
  inline Eigen::VectorXd minDeltaInput() const;
  inline Eigen::VectorXd maxDeltaInput() const;

  /* ピッチ回転のトリムに用いる舵面の添字 */
  inline const size_t& elevatorChannel() const;
  inline const size_t& inputSize() const;

  inline const Eigen::Matrix<double, kStateSize, kStateSize>& A() const;
  inline const Eigen::Matrix<double, kStateSize, Eigen::Dynamic>& B() const;

  // X_u (2.2-36)
  inline const double& u_u() const;
  // X_alpha (2.2-36)
  inline const double& u_alpha() const;
  inline const double& u_q() const;
  inline const double& u_theta() const;

  // Z_u_bar (2.2-37)
  inline const double& alpha_u() const;
  // Z_alpha_bar (2.2-37)
  inline const double& alpha_alpha() const;
  inline const double& alpha_q() const;
  inline const double& alpha_theta() const;

  // Y_beta_bar (3.2-20)
  inline const double& beta_beta() const;
  inline const double& beta_p() const;
  inline const double& beta_r() const;
  inline const double& beta_phi() const;

  inline const double& phi_beta() const;
  inline const double& phi_p() const;
  inline const double& phi_r() const;
  inline const double& phi_phi() const;

  inline const double& theta_u() const;
  inline const double& theta_alpha() const;
  inline const double& theta_q() const;
  inline const double& theta_theta() const;

  // L_beta_dash (3.2-21)
  inline const double& p_beta() const;
  // L_p_dash (3.2-21)
  inline const double& p_p() const;
  // L_r_dash (3.2-21)
  inline const double& p_r() const;
  inline const double& p_phi() const;

  // M_u_dash (2.2-39)
  inline const double& q_u() const;
  // M_alpha_dash (2.2-39)
  inline const double& q_alpha() const;
  // M_q_dash (2.2-39)
  inline const double& q_q() const;
  // M_theta_dash (2.2-39)
  inline const double& q_theta() const;

  // N_beta_dash (3.2-22)
  inline const double& r_beta() const;
  // N_p_dash (3.2-22)
  inline const double& r_p() const;
  // N_r_dash (3.2-22)
  inline const double& r_r() const;
  inline const double& r_phi() const;

  inline double u_thrust() const;

  // Z_delta_bar (2.2-37)
  inline const double& alpha_delta(const size_t& cs_idx) const;
  // Y_delta_bar (3.2-20)
  inline const double& beta_delta(const size_t& cs_idx) const;
  // L_delta_bar (3.2-21)
  inline const double& p_delta(const size_t& cs_idx) const;
  // M_delta_bar (2.2-38)
  inline const double& q_delta(const size_t& cs_idx) const;
  // N_delta_bar (3.2.22)
  inline const double& r_delta(const size_t& cs_idx) const;

private:
  const Drone& drone_;
  const kdl::Tree& tree_;

  kdl::TreeFkSolverPos fk_solver_;
  kdl::TreeJntToInertiaSolver inertia_solver_;
  RotorAxisExtractor x_rotors_;
  TrimConditions trim_;

  size_t u_size_;
  Eigen::VectorXd min_u_;                     // 制御入力の最小値
  Eigen::VectorXd max_u_;                     // 制御入力の最大値
  Eigen::Matrix<double, kStateSize, 1> x_0_;  // トリム時の状態
  Eigen::VectorXd u_0_;                       // トリム時の制御入力

  // 各係数のバッファ
  Eigen::Matrix<double, kStateSize, kStateSize> A_;
  Eigen::Matrix<double, kStateSize, Eigen::Dynamic> B_;  // 列数は舵面数と一致

  void setInputLimits(const double& battery_voltage);
};

inline const TrimConditions& MicroDisturbanceEoM::trimCondition() const
{
  return trim_;
}

inline const StabilityDerivativesCG& MicroDisturbanceEoM::stabilityDerivativesCG() const
{
  return trim_.stabilityDerivativesCG();
}

inline const MicroDisturbanceEoM::StateVector& MicroDisturbanceEoM::trimState() const
{
  return x_0_;
}

inline const Eigen::VectorXd& MicroDisturbanceEoM::trimInput() const
{
  return u_0_;
}

inline const Eigen::VectorXd& MicroDisturbanceEoM::minInput() const
{
  return min_u_;
}

inline const Eigen::VectorXd& MicroDisturbanceEoM::maxInput() const
{
  return max_u_;
}

inline Eigen::VectorXd MicroDisturbanceEoM::minDeltaInput() const
{
  return min_u_ - u_0_;
}

inline Eigen::VectorXd MicroDisturbanceEoM::maxDeltaInput() const
{
  return max_u_ - u_0_;
}

inline const size_t& MicroDisturbanceEoM::elevatorChannel() const
{
  return trim_.elevatorChannel();
}

inline const size_t& MicroDisturbanceEoM::inputSize() const
{
  return u_size_;
}

inline const MicroDisturbanceEoM::StateMatrix& MicroDisturbanceEoM::A() const
{
  return A_;
}

inline const Eigen::Matrix<double, MicroDisturbanceEoM::kStateSize, Eigen::Dynamic>& MicroDisturbanceEoM::B() const
{
  return B_;
}

inline const double& MicroDisturbanceEoM::u_u() const
{
  return A_(kStateIdx_u, kStateIdx_u);
}

inline const double& MicroDisturbanceEoM::u_alpha() const
{
  return A_(kStateIdx_u, kStateIdx_alpha);
}

inline const double& MicroDisturbanceEoM::u_q() const
{
  return A_(kStateIdx_u, kStateIdx_q);
}

inline const double& MicroDisturbanceEoM::u_theta() const
{
  return A_(kStateIdx_u, kStateIdx_theta);
}

inline const double& MicroDisturbanceEoM::alpha_u() const
{
  return A_(kStateIdx_alpha, kStateIdx_u);
}

inline const double& MicroDisturbanceEoM::alpha_alpha() const
{
  return A_(kStateIdx_alpha, kStateIdx_alpha);
}

inline const double& MicroDisturbanceEoM::alpha_q() const
{
  return A_(kStateIdx_alpha, kStateIdx_q);
}

inline const double& MicroDisturbanceEoM::alpha_theta() const
{
  return A_(kStateIdx_alpha, kStateIdx_theta);
}

inline const double& MicroDisturbanceEoM::beta_beta() const
{
  return A_(kStateIdx_beta, kStateIdx_beta);
}

inline const double& MicroDisturbanceEoM::beta_p() const
{
  return A_(kStateIdx_beta, kStateIdx_p);
}

inline const double& MicroDisturbanceEoM::beta_r() const
{
  return A_(kStateIdx_beta, kStateIdx_r);
}

inline const double& MicroDisturbanceEoM::beta_phi() const
{
  return A_(kStateIdx_beta, kStateIdx_phi);
}

inline const double& MicroDisturbanceEoM::phi_beta() const
{
  return A_(kStateIdx_phi, kStateIdx_beta);
}

inline const double& MicroDisturbanceEoM::phi_p() const
{
  return A_(kStateIdx_phi, kStateIdx_p);
}

inline const double& MicroDisturbanceEoM::phi_r() const
{
  return A_(kStateIdx_phi, kStateIdx_r);
}

inline const double& MicroDisturbanceEoM::phi_phi() const
{
  return A_(kStateIdx_phi, kStateIdx_phi);
}

inline const double& MicroDisturbanceEoM::theta_u() const
{
  return A_(kStateIdx_theta, kStateIdx_u);
}

inline const double& MicroDisturbanceEoM::theta_alpha() const
{
  return A_(kStateIdx_theta, kStateIdx_alpha);
}

inline const double& MicroDisturbanceEoM::theta_q() const
{
  return A_(kStateIdx_theta, kStateIdx_q);
}

inline const double& MicroDisturbanceEoM::theta_theta() const
{
  return A_(kStateIdx_theta, kStateIdx_theta);
}

inline const double& MicroDisturbanceEoM::p_beta() const
{
  return A_(kStateIdx_p, kStateIdx_beta);
}

inline const double& MicroDisturbanceEoM::p_p() const
{
  return A_(kStateIdx_p, kStateIdx_p);
}

inline const double& MicroDisturbanceEoM::p_r() const
{
  return A_(kStateIdx_p, kStateIdx_r);
}

inline const double& MicroDisturbanceEoM::p_phi() const
{
  return A_(kStateIdx_p, kStateIdx_phi);
}

inline const double& MicroDisturbanceEoM::q_u() const
{
  return A_(kStateIdx_q, kStateIdx_u);
}

inline const double& MicroDisturbanceEoM::q_alpha() const
{
  return A_(kStateIdx_q, kStateIdx_alpha);
}

inline const double& MicroDisturbanceEoM::q_q() const
{
  return A_(kStateIdx_q, kStateIdx_q);
}

inline const double& MicroDisturbanceEoM::q_theta() const
{
  return A_(kStateIdx_q, kStateIdx_theta);
}

inline const double& MicroDisturbanceEoM::r_beta() const
{
  return A_(kStateIdx_r, kStateIdx_beta);
}

inline const double& MicroDisturbanceEoM::r_p() const
{
  return A_(kStateIdx_r, kStateIdx_p);
}

inline const double& MicroDisturbanceEoM::r_r() const
{
  return A_(kStateIdx_r, kStateIdx_r);
}

inline const double& MicroDisturbanceEoM::r_phi() const
{
  return A_(kStateIdx_r, kStateIdx_phi);
}

inline double MicroDisturbanceEoM::u_thrust() const
{
  return 1 / inertia_solver_.getInertia().getMass();
}

inline const double& MicroDisturbanceEoM::alpha_delta(const size_t& cs_idx) const
{
  return B_(kStateIdx_alpha, x_rotors_.count() + cs_idx);
}

inline const double& MicroDisturbanceEoM::beta_delta(const size_t& cs_idx) const
{
  return B_(kStateIdx_beta, x_rotors_.count() + cs_idx);
}

inline const double& MicroDisturbanceEoM::p_delta(const size_t& cs_idx) const
{
  return B_(kStateIdx_p, x_rotors_.count() + cs_idx);
}

inline const double& MicroDisturbanceEoM::q_delta(const size_t& cs_idx) const
{
  return B_(kStateIdx_q, x_rotors_.count() + cs_idx);
}

inline const double& MicroDisturbanceEoM::r_delta(const size_t& cs_idx) const
{
  return B_(kStateIdx_r, x_rotors_.count() + cs_idx);
}
}  // namespace tobas
