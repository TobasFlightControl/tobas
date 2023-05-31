#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define POS_IDX (0)
#define ALT_IDX (POS_IDX + 2)
#define VEL_IDX (POS_IDX + 3)
#define QUAT_IDX (VEL_IDX + 3)
#define AB_IDX (QUAT_IDX + 4)
#define WB_IDX (AB_IDX + 3)
#define STATE_SIZE (WB_IDX + 3)

#define dPOS_IDX (0)
#define dALT_IDX (dPOS_IDX + 2)
#define dVEL_IDX (dPOS_IDX + 3)
#define dTHETA_IDX (dVEL_IDX + 3)
#define dAB_IDX (dTHETA_IDX + 3)
#define dWB_IDX (dAB_IDX + 3)
#define dSTATE_SIZE (dWB_IDX + 3)

// the main ESKF class
class ErrorStateKalmanFilter
{
  using StateMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;
  using StateVector = Eigen::Matrix<double, STATE_SIZE, 1>;
  using dStateMatrix = Eigen::Matrix<double, dSTATE_SIZE, dSTATE_SIZE>;
  using dStateVector = Eigen::Matrix<double, dSTATE_SIZE, 1>;
  using Scalar = Eigen::Matrix<double, 1, 1>;

public:
  explicit ErrorStateKalmanFilter();

  void initialize(
    const Eigen::Vector3d& a_grav,
    const StateVector& init_state,
    const dStateMatrix& init_P,
    double var_acc,
    double var_omega,
    double var_acc_bias,
    double var_omega_bias);

  // Concatenates relevant vectors to one large vector.
  static StateVector makeState(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& v,
    const Eigen::Quaterniond& q,
    const Eigen::Vector3d& a_b,
    const Eigen::Vector3d& w_b);

  // Inserts relevant parts of the block-diagonal of the P matrix
  static dStateMatrix makeP(
    const Eigen::Matrix3d& cov_pos,
    const Eigen::Matrix3d& cov_vel,
    const Eigen::Matrix3d& cov_dtheta,
    const Eigen::Matrix3d& cov_a_b,
    const Eigen::Matrix3d& cov_w_b);

  // The quaternion convention in the document is "Hamilton" convention.
  // Eigen has a different order of components, so we need conversion
  static Eigen::Quaterniond quatFromHamilton(const Eigen::Vector4d& qHam);
  static Eigen::Vector4d quatToHamilton(const Eigen::Quaterniond& q);
  static Eigen::Matrix3d rotVecToMat(const Eigen::Vector3d& in);
  static Eigen::Quaterniond rotVecToQuat(const Eigen::Vector3d& in);
  static Eigen::Vector3d quatToRotVec(const Eigen::Quaterniond& q);
  static Eigen::Matrix3d getSkew(const Eigen::Vector3d& in);

  // Acessors of nominal state
  inline Eigen::Vector3d getPosition3D() const
  {
    return nominal_state_.block<3, 1>(POS_IDX, 0);
  }
  inline Eigen::Vector2d getPosition2D() const
  {
    return nominal_state_.block<2, 1>(POS_IDX, 0);
  }
  inline double getAltitude() const
  {
    return nominal_state_(ALT_IDX);
  }
  inline Eigen::Vector3d getVelocity() const
  {
    return nominal_state_.block<3, 1>(VEL_IDX, 0);
  }
  inline Eigen::Quaterniond getQuaternion() const
  {
    return quatFromHamilton(getQuatVector());
  }
  inline Eigen::Vector3d getAccelBias() const
  {
    return nominal_state_.block<3, 1>(AB_IDX, 0);
  }
  inline Eigen::Vector3d getGyroBias() const
  {
    return nominal_state_.block<3, 1>(WB_IDX, 0);
  }

  void predictIMU(const Eigen::Vector3d& a_m, const Eigen::Vector3d& omega_m, const double dt);

  void measurePosition3D(const Eigen::Vector3d& pos_meas, const Eigen::Matrix3d& pos_cov);
  void measurePosition2D(const Eigen::Vector2d& xy_meas, const Eigen::Matrix2d& xy_cov);
  void measureAltitude(const double& z_meas, const double& z_var);
  void measureVelocity(const Eigen::Vector3d& vel_meas, const Eigen::Matrix3d& vel_cov);
  void measureQuaternion(const Eigen::Quaterniond& q_meas, const Eigen::Matrix3d& theta_cov);

  Eigen::Matrix3d getDCM();

private:
  double var_acc_;
  double var_omega_;
  double var_acc_bias_;
  double var_omega_bias_;

  Eigen::Vector3d a_grav_;                    // Acceleration due to gravity in global frame [m/s^2]
  StateVector nominal_state_;                 // State vector of the filter
  dStateMatrix P_;                            // Covariance of the error state
  dStateMatrix F_x_;                          // Jacobian of the state transition

  Eigen::Matrix<double, 4, 3> getQ_dtheta();  // eqn 280, page 62

  template <size_t M>
  void correct(
    const Eigen::Matrix<double, M, 1>& delta_meas,
    const Eigen::Matrix<double, M, M>& meas_cov,
    const Eigen::Matrix<double, M, dSTATE_SIZE>& H);

  void injectErrorState(const dStateVector& error_state);

  // クオータニオンをベクトルの形で得る．(w,x,y,z)の順であることに注意！x()などのメソッドでアクセスするとずれる！
  inline Eigen::Vector4d getQuatVector() const
  {
    return nominal_state_.block<4, 1>(QUAT_IDX, 0);
  }
};

template <size_t M>
void ErrorStateKalmanFilter::correct(
  const Eigen::Matrix<double, M, 1>& delta_meas,
  const Eigen::Matrix<double, M, M>& meas_cov,
  const Eigen::Matrix<double, M, dSTATE_SIZE>& H)
{
  // Kalman gain
  const auto PHt = P_ * H.transpose();
  const auto K = PHt * (H * PHt + meas_cov).inverse();

  // Correction error state
  const auto error_state = K * delta_meas;
  const auto I_KH = dStateMatrix::Identity() - K * H;

  // Update P (simple form)
  // P_ = I_KH * P_;  // Simple form
  P_ = I_KH * P_ * I_KH.transpose() + K * meas_cov * K.transpose();  // Joseph form

  injectErrorState(error_state);
}
