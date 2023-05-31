#include "../../include/state_estimation_eskf/eskf.hpp"
#include "../../include/state_estimation_eskf/unrolled_joseph.hpp"

#define SQ(x) (x * x)
#define I_3 (Matrix3d::Identity())

using namespace std;
using namespace Eigen;

ErrorStateKalmanFilter::ErrorStateKalmanFilter()
{
}

void ErrorStateKalmanFilter::initialize(
  const Vector3d& a_grav,
  const StateVector& init_state,
  const dStateMatrix& init_P,
  double var_acc,
  double var_omega,
  double var_acc_bias,
  double var_omega_bias)
{
  var_acc_ = var_acc;
  var_omega_ = var_omega;
  var_acc_bias_ = var_acc_bias;
  var_omega_bias_ = var_omega_bias;
  a_grav_ = a_grav;
  nominal_state_ = init_state;
  P_ = init_P;

  // Jacobian of the state transition: page 59, eqn 269
  // Precompute constant part only
  F_x_.setZero();
  // dPos row
  F_x_.block<3, 3>(dPOS_IDX, dPOS_IDX).diagonal().fill(1.);
  // dVel row
  F_x_.block<3, 3>(dVEL_IDX, dVEL_IDX).diagonal().fill(1.);
  // dTheta row
  // dAccelBias row
  F_x_.block<3, 3>(dAB_IDX, dAB_IDX).diagonal().fill(1.);
  // dGyroBias row
  F_x_.block<3, 3>(dWB_IDX, dWB_IDX).diagonal().fill(1.);
}

Matrix<double, STATE_SIZE, 1> ErrorStateKalmanFilter::makeState(
  const Vector3d& p,
  const Vector3d& v,
  const Quaterniond& q,
  const Vector3d& a_b,
  const Vector3d& w_b)
{
  StateVector out;
  out << p, v, quatToHamilton(q).normalized(), a_b, w_b;
  return out;
}

Matrix<double, dSTATE_SIZE, dSTATE_SIZE> ErrorStateKalmanFilter::makeP(
  const Matrix3d& cov_pos,
  const Matrix3d& cov_vel,
  const Matrix3d& cov_dtheta,
  const Matrix3d& cov_a_b,
  const Matrix3d& cov_w_b)
{
  dStateMatrix P;
  P.setZero();
  P.block<3, 3>(dPOS_IDX, dPOS_IDX) = cov_pos;
  P.block<3, 3>(dVEL_IDX, dVEL_IDX) = cov_vel;
  P.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = cov_dtheta;
  P.block<3, 3>(dAB_IDX, dAB_IDX) = cov_a_b;
  P.block<3, 3>(dWB_IDX, dWB_IDX) = cov_w_b;
  return P;
}

Matrix3d ErrorStateKalmanFilter::getDCM()
{
  return getQuaternion().matrix();
}

Quaterniond ErrorStateKalmanFilter::quatFromHamilton(const Vector4d& qHam)
{
  return Quaterniond((Vector4d() << qHam.block<3, 1>(1, 0),  // x, y, z
                      qHam.block<1, 1>(0, 0)                 // w
                      )
                       .finished());
}

Vector4d ErrorStateKalmanFilter::quatToHamilton(const Quaterniond& q)
{
  return (Vector4d() << q.coeffs().block<1, 1>(3, 0),  // w
          q.coeffs().block<3, 1>(0, 0)                 // x, y, z
          )
    .finished();
}

Matrix3d ErrorStateKalmanFilter::getSkew(const Vector3d& in)
{
  Matrix3d out;
  out << 0, -in(2), in(1), in(2), 0, -in(0), -in(1), in(0), 0;
  return out;
}

Matrix3d ErrorStateKalmanFilter::rotVecToMat(const Vector3d& in)
{
  const auto angle = in.norm();
  const auto axis = (angle == 0) ? Vector3d(1, 0, 0) : in.normalized();
  AngleAxisd angle_axis(angle, axis);
  return angle_axis.toRotationMatrix();
}

Quaterniond ErrorStateKalmanFilter::rotVecToQuat(const Vector3d& in)
{
  const auto angle = in.norm();
  Vector3d axis = (angle == 0) ? Vector3d(1, 0, 0) : in.normalized();
  return Quaterniond(AngleAxisd(angle, axis));
}

Vector3d ErrorStateKalmanFilter::quatToRotVec(const Quaterniond& q)
{
  AngleAxisd angle_axis(q);
  return angle_axis.angle() * angle_axis.axis();
}

void ErrorStateKalmanFilter::predictIMU(
  const Vector3d& a_m,
  const Vector3d& omega_m,
  const double dt)
{
  // DCM of current state
  const auto Rot = getDCM();
  // Accelerometer measurement
  const auto acc_body = a_m - getAccelBias();
  const auto acc_global = Rot * acc_body;
  // Gyro measruement
  const auto omega = omega_m - getGyroBias();
  const auto delta_theta = omega * dt;
  const auto q_delta_theta = rotVecToQuat(delta_theta);
  const auto R_delta_theta = q_delta_theta.toRotationMatrix();

  // Nominal state kinematics (eqn 259, pg 58)
  const auto delta_pos = getVelocity() * dt + 0.5 * (acc_global + a_grav_) * SQ(dt);
  nominal_state_.block<3, 1>(POS_IDX, 0) += delta_pos;
  nominal_state_.block<3, 1>(VEL_IDX, 0) += (acc_global + a_grav_) * dt;
  nominal_state_.block<4, 1>(QUAT_IDX, 0) =
    quatToHamilton(getQuaternion() * q_delta_theta).normalized();

  // Jacobian of the state transition (eqn 269, page 59). Update dynamic parts only.
  // dPos row
  F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX).diagonal().fill(dt);  // = I_3 * dt
  // dVel row
  F_x_.block<3, 3>(dVEL_IDX, dTHETA_IDX) = -Rot * getSkew(acc_body) * dt;
  F_x_.block<3, 3>(dVEL_IDX, dAB_IDX) = -Rot * dt;
  // dTheta row
  F_x_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = R_delta_theta.transpose();
  F_x_.block<3, 3>(dTHETA_IDX, dWB_IDX).diagonal().fill(-dt);  // = -I_3 * dt;

  // Predict P and inject variance (with diagonal optimization)
  P_ = F_x_ * P_ * F_x_.transpose();

  // dStateMatrix Pnew;
  // unrolledFPFt(P_, Pnew, dt, -Rot * getSkew(acc_body) * dt, -Rot * dt,
  // R_delta_theta.transpose()); P_ = Pnew;

  // Inject process noise
  P_.diagonal().block<3, 1>(dVEL_IDX, 0).array() += var_acc_ * SQ(dt);
  P_.diagonal().block<3, 1>(dTHETA_IDX, 0).array() += var_omega_ * SQ(dt);
  P_.diagonal().block<3, 1>(dAB_IDX, 0).array() += var_acc_bias_ * dt;
  P_.diagonal().block<3, 1>(dWB_IDX, 0).array() += var_omega_bias_ * dt;
}

Matrix<double, 4, 3> ErrorStateKalmanFilter::getQ_dtheta()
{
  Vector4d qby2 = 0.5 * getQuatVector();
  // Assing to letters for readability. Note Hamilton order.
  const auto w = qby2[0];
  const auto x = qby2[1];
  const auto y = qby2[2];
  const auto z = qby2[3];
  Matrix<double, 4, 3> Q_dtheta;
  Q_dtheta << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
  return Q_dtheta;
}

void ErrorStateKalmanFilter::measurePosition3D(const Vector3d& pos_meas, const Matrix3d& pos_cov)
{
  const auto delta_pos = pos_meas - getPosition3D();

  // H is a trivial observation of purely the position
  Matrix<double, 3, dSTATE_SIZE> H;
  H.setZero();
  H.block<3, 3>(0, dPOS_IDX).diagonal().fill(1.);

  // Apply update
  correct<3>(delta_pos, pos_cov, H);
}

void ErrorStateKalmanFilter::measurePosition2D(const Vector2d& xy_meas, const Matrix2d& xy_cov)
{
  const auto delta_xy = xy_meas - getPosition2D();

  // H is a trivial observation of purely the position
  Matrix<double, 2, dSTATE_SIZE> H;
  H.setZero();
  H.block<2, 2>(0, dPOS_IDX).diagonal().fill(1.);

  // Apply update
  correct<2>(delta_xy, xy_cov, H);
}

void ErrorStateKalmanFilter::measureAltitude(const double& z_meas, const double& z_cov)
{
  const auto delta_z = z_meas - getAltitude();

  // H is a trivial observation of purely the position
  Matrix<double, 1, dSTATE_SIZE> H;
  H.setZero();
  H(0, dALT_IDX) = 1.;

  // Apply update
  correct<1>(Scalar(delta_z), Scalar(z_cov), H);
}

void ErrorStateKalmanFilter::measureVelocity(const Vector3d& vel_meas, const Matrix3d& vel_cov)
{
  const auto delta_vel = vel_meas - getVelocity();

  // H is a trivial observation of purely the velocity
  Matrix<double, 3, dSTATE_SIZE> H;
  H.setZero();
  H.block<3, 3>(0, dVEL_IDX).diagonal().fill(1.);

  // Apply update
  correct<3>(delta_vel, vel_cov, H);
}

void ErrorStateKalmanFilter::measureQuaternion(
  const Quaterniond& q_gb_meas,
  const Matrix3d& theta_cov)
{
  const auto q_gb_nominal = getQuaternion();
  const auto q_bNominal_bMeas = q_gb_nominal.conjugate() * q_gb_meas;
  const auto delta_theta = quatToRotVec(q_bNominal_bMeas);

  // Because of the above construction, H is a trivial observation of dtheta
  Matrix<double, 3, dSTATE_SIZE> H;
  H.setZero();
  H.block<3, 3>(0, dTHETA_IDX).diagonal().fill(1.);

  // Apply update
  correct<3>(delta_theta, theta_cov, H);
}

void ErrorStateKalmanFilter::injectErrorState(const dStateVector& error_state)
{  // Inject error state into nominal state (eqn 282, pg 62)
  nominal_state_.block<3, 1>(POS_IDX, 0) += error_state.block<3, 1>(dPOS_IDX, 0);
  nominal_state_.block<3, 1>(VEL_IDX, 0) += error_state.block<3, 1>(dVEL_IDX, 0);
  Vector3d dtheta = error_state.block<3, 1>(dTHETA_IDX, 0);
  Quaterniond q_dtheta = rotVecToQuat(dtheta);
  nominal_state_.block<4, 1>(QUAT_IDX, 0) = quatToHamilton(getQuaternion() * q_dtheta).normalized();
  nominal_state_.block<3, 1>(AB_IDX, 0) += error_state.block<3, 1>(dAB_IDX, 0);
  nominal_state_.block<3, 1>(WB_IDX, 0) += error_state.block<3, 1>(dWB_IDX, 0);

  // Reflect this tranformation in the P matrix, aka ErrorStateKalmanFilter Reset
  // Note that the document suggests that this step is optional
  // eqn 287, pg 63
  const auto G_theta = I_3 - getSkew(0.5 * dtheta);
  P_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) =
    G_theta * P_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) * G_theta.transpose();
}
