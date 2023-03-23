#pragma once

#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "./l_time.hpp"

#define SUPPORT_STDIOSTREAM

#define POS_IDX (0)
#define VEL_IDX (POS_IDX + 3)
#define QUAT_IDX (VEL_IDX + 3)
#define AB_IDX (QUAT_IDX + 4)
#define GB_IDX (AB_IDX + 3)
#define STATE_SIZE (GB_IDX + 3)

#define dPOS_IDX (0)
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
    noMethod,
    // Keep buffer of states, calculate what the update would have been, and apply to current state.
    applyUpdateToNew,
    // Method as described by Larson et al. Though a buffer of IMU values is kept, and a single
    // update taking the average of these values is used.
    larsonAverageIMU,
    // As above, though no buffer kept, use most recent value as representing the average.
    larsonNewestIMU,
    // As above, though the buffer is applied with the correct time steps, fully as described by
    // Larson.
    larsonFull,
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
  inline Eigen::Vector3d getPos() const
  {
    return nominal_state_.block<3, 1>(POS_IDX, 0);
  }
  inline Eigen::Vector3d getVel() const
  {
    return nominal_state_.block<3, 1>(VEL_IDX, 0);
  }
  inline Eigen::Quaterniond getQuat() const
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
  void measurePos(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Matrix3d& pos_cov,
    lTime stamp,
    lTime now);

  // Called when there is a new measurment from an absolute velocity reference.
  // Note that this has no body offset, i.e. it assumes exact observation of the center of the IMU.
  void measureVel(
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix3d& vel_cov,
    lTime stamp,
    lTime now);

  // Called when there is a new measurment from an absolute orientation reference.
  // The uncertianty is represented as the covariance of a rotation vector in the body frame
  void measureQuat(
    const Eigen::Quaterniond& q_meas,
    const Eigen::Matrix3d& theta_cov,
    lTime stamp,
    lTime now);

  Eigen::Matrix3d getDCM();

private:
  Eigen::Matrix<double, 4, 3> getQ_dtheta();  // eqn 280, page 62

  void update_3D(
    const Eigen::Vector3d& delta_measurement,
    const Eigen::Matrix3d& meas_covariance,
    const Eigen::Matrix<double, 3, dSTATE_SIZE>& H,
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

  // IMU Noise values, used in prediction
  double var_acc_;
  double var_omega_;
  double var_acc_bias_;
  double var_omega_bias_;
  Eigen::Vector3d a_grav_;     // Acceleration due to gravity in global frame [m/s^2]
  StateVector nominal_state_;  // State vector of the filter
  dStateMatrix P_;             // Covariance of the error state
  // Jacobian of the state transition: page 59, eqn 269
  // Note that we precompute the static parts in the constructor,
  // and update the dynamic parts in the predict function
  dStateMatrix F_x_;

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
};
