#pragma once

#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "./l_time.hpp"

#define SUPPORT_STDIOSTREAM

#define POS_IDX (0)
#define ALT_IDX (POS_IDX + 2)
#define VEL_IDX (POS_IDX + 3)
#define QUAT_IDX (VEL_IDX + 3)
#define AB_IDX (QUAT_IDX + 4)
#define GB_IDX (AB_IDX + 3)
#define STATE_SIZE (GB_IDX + 3)

#define dPOS_IDX (0)
#define dALT_IDX (dPOS_IDX + 2)
#define dVEL_IDX (dPOS_IDX + 3)
#define dTHETA_IDX (dVEL_IDX + 3)
#define dAB_IDX (dTHETA_IDX + 3)
#define dGB_IDX (dAB_IDX + 3)
#define dSTATE_SIZE (dGB_IDX + 3)

// the main ESKF class
class ErrorStateKalmanFilter
{
  using StateMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;
  using StateVector = Eigen::Matrix<double, STATE_SIZE, 1>;
  using dStateMatrix = Eigen::Matrix<double, dSTATE_SIZE, dSTATE_SIZE>;
  using dStateVector = Eigen::Matrix<double, dSTATE_SIZE, 1>;
  using Scalar = Eigen::Matrix<double, 1, 1>;

public:
  enum DelayType
  {
    // apply updates  as if they are new.
    NO_METHOD,
    // Keep buffer of states, calculate what the update would have been, and apply to current state.
    APPLY_UPDATE_TO_NEW,
    // Method as described by Larson et al. Though a buffer of IMU values is kept, and a single
    // update taking the average of these values is used.
    LARSON_AVERATE_IMU,
    // As above, though no buffer kept, use most recent value as representing the average.
    LARSON_NEWEST_IMU,
    // As above, though the buffer is applied with the correct time steps, fully as described by
    // Larson.
    LARSON_FULL,
  };

  struct ImuMeasurement
  {
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    lTime time;
  };

  ErrorStateKalmanFilter();

  void initialize(
    Eigen::Vector3d a_grav,
    const StateVector& init_state,
    const dStateMatrix& init_P,
    double var_acc,
    double var_omega,
    double var_acc_bias,
    double var_omega_bias,
    DelayType delay_handling,
    int buf_length);

  // Concatenates relevant vectors to one large vector.
  static StateVector makeState(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& v,
    const Eigen::Quaterniond& q,
    const Eigen::Vector3d& a_b,
    const Eigen::Vector3d& omega_b);
  // Inserts relevant parts of the block-diagonal of the P matrix
  static dStateMatrix makeP(
    const Eigen::Matrix3d& cov_pos,
    const Eigen::Matrix3d& cov_vel,
    const Eigen::Matrix3d& cov_dtheta,
    const Eigen::Matrix3d& cov_a_b,
    const Eigen::Matrix3d& cov_omega_b);

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
    return nominal_state_.block<3, 1>(GB_IDX, 0);
  }

  // Called when there is a new measurment from the IMU.
  // dt is the integration time of this sample, nominally the IMU sample period
  void predictIMU(
    const Eigen::Vector3d& a_m,
    const Eigen::Vector3d& omega_m,
    const double dt,
    lTime stamp);

  // Called when there is a new measurment from an absolute position reference.
  // Note that this has no body offset, i.e. it assumes exact observation of the center of the IMU.
  void measurePosition3D(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Matrix3d& pos_cov,
    lTime stamp,
    lTime now);

  void measurePosition2D(
    const Eigen::Vector2d& xy_meas,
    const Eigen::Matrix2d& xy_cov,
    lTime stamp,
    lTime now);

  void measureAltitude(const double& z_meas, const double& z_var, lTime stamp, lTime now);

  // Called when there is a new measurment from an absolute velocity reference.
  // Note that this has no body offset, i.e. it assumes exact observation of the center of the IMU.
  void measureVelocity(
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix3d& vel_cov,
    lTime stamp,
    lTime now);

  // Called when there is a new measurment from an absolute orientation reference.
  // The uncertianty is represented as the covariance of a rotation vector in the body frame
  void measureQuaternion(
    const Eigen::Quaterniond& q_meas,
    const Eigen::Matrix3d& theta_cov,
    lTime stamp,
    lTime now);

  Eigen::Matrix3d getDCM();

private:
  double var_acc_;
  double var_omega_;
  double var_acc_bias_;
  double var_omega_bias_;

  Eigen::Vector3d a_grav_;     // Acceleration due to gravity in global frame [m/s^2]
  StateVector nominal_state_;  // State vector of the filter
  dStateMatrix P_;             // Covariance of the error state
  dStateMatrix F_x_;           // Jacobian of the state transition

  DelayType delay_handling_;
  int buf_length_;
  int recent_ptr_;
  // pointers to structures that are allocated only after choosing a time delay handling method.
  std::vector<std::pair<lTime, StateVector>>* state_hist_ptr_;
  std::vector<std::pair<lTime, dStateMatrix>>* P_hist_ptr_;
  std::vector<ImuMeasurement>* imu_hist_ptr_;
  ImuMeasurement last_imu_;
  lTime first_meas_time_;
  lTime last_meas_;
  dStateMatrix* M_ptr_;

  Eigen::Matrix<double, 4, 3> getQ_dtheta();  // eqn 280, page 62

  template <size_t M>
  void correct(
    const Eigen::Matrix<double, M, 1>& delta_meas,
    const Eigen::Matrix<double, M, M>& meas_cov,
    const Eigen::Matrix<double, M, dSTATE_SIZE>& H,
    lTime stamp,
    lTime now);

  void injectErrorState(const dStateVector& error_state);

  // get best time from history of state
  int getClosestTime(std::vector<std::pair<lTime, StateVector>>* ptr, lTime stamp);

  // get best time from history of imu
  int getClosestTime(std::vector<ImuMeasurement>* ptr, lTime stamp);

  ImuMeasurement getAverageIMU(lTime stamp);

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
  const Eigen::Matrix<double, M, dSTATE_SIZE>& H,
  lTime stamp,
  lTime now)
{
  // generate M matrix for time correction methods
  int best_time_idx;
  bool is_normal_pass = true;
  if (delay_handling_ == LARSON_AVERATE_IMU)
  {
    if (stamp > first_meas_time_)
    {
      is_normal_pass = false;
    }
  }
  if (delay_handling_ == LARSON_AVERATE_IMU && !is_normal_pass)
  {
    ImuMeasurement average_imu = getAverageIMU(stamp);
    double dt = (now - stamp).toSec();
    Eigen::Vector3d acc_body = average_imu.acc - getAccelBias();
    Eigen::Vector3d omega = average_imu.gyro - getGyroBias();
    Eigen::Vector3d delta_theta = omega * dt;
    Eigen::Quaterniond q_delta_theta = rotVecToQuat(delta_theta);
    Eigen::Matrix3d R_delta_theta = q_delta_theta.toRotationMatrix();
    best_time_idx = getClosestTime(state_hist_ptr_, stamp);

    Eigen::Matrix3d Rot =
      quatFromHamilton(state_hist_ptr_->at(best_time_idx).second.block<4, 1>(QUAT_IDX, 0)).matrix();
    // dPos row
    F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX).diagonal().fill(dt);  // = I_3 * _dt
    // dVel row
    F_x_.block<3, 3>(dVEL_IDX, dTHETA_IDX) = -Rot * getSkew(acc_body) * dt;
    F_x_.block<3, 3>(dVEL_IDX, dAB_IDX) = -Rot * dt;
    // dTheta row
    F_x_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = R_delta_theta.transpose();
    F_x_.block<3, 3>(dTHETA_IDX, dGB_IDX).diagonal().fill(-dt);  // = -I_3 * dt;
  }

  // Kalman gain
  Eigen::Matrix<double, dSTATE_SIZE, M> PHt = P_ * H.transpose();
  Eigen::Matrix<double, dSTATE_SIZE, M> K;
  if ((delay_handling_ == NO_METHOD || delay_handling_ == APPLY_UPDATE_TO_NEW
       || delay_handling_ == LARSON_AVERATE_IMU))
  {
    K = PHt * (H * PHt + meas_cov).inverse();
  }
  if (delay_handling_ == LARSON_AVERATE_IMU && !is_normal_pass)
  {
    K = F_x_ * K;
  }

  // Correction error state
  dStateVector errorState = K * delta_meas;

  // Update P (simple form)
  // P_ = (dStateMatrix::Identity() - K * H) * P_;
  // Update P (Joseph form)
  dStateMatrix I_KH = dStateMatrix::Identity() - K * H;
  if (delay_handling_ == NO_METHOD || delay_handling_ == APPLY_UPDATE_TO_NEW)
  {
    P_ = I_KH * P_ * I_KH.transpose() + K * meas_cov * K.transpose();
  }
  if (delay_handling_ == LARSON_AVERATE_IMU && !is_normal_pass)
  {
    P_ = P_ - K * H * P_hist_ptr_->at(best_time_idx).second * F_x_;
  }

  injectErrorState(errorState);
}
