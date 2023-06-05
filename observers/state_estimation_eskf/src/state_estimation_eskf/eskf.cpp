#include <dh_eigen_tools/linalg.hpp>

#include "../../include/state_estimation_eskf/eskf.hpp"

#define SQ(x) (x * x)
#define I_3 (Matrix3d::Identity())

using namespace std;
using namespace Eigen;

ErrorStateKalmanFilter::ErrorStateKalmanFilter()
{
}

void ErrorStateKalmanFilter::initialize(
  double var_acc,
  double var_gyro,
  double var_acc_bias,
  double var_gyro_bias,
  const Vector3d& grav_W,
  const Vector3d& mag_W,
  const Vector3d& init_pos,
  const Vector3d& init_vel,
  const Quaterniond& init_quat,
  const Matrix3d& cov_pos,
  const Matrix3d& cov_vel,
  const Matrix3d& cov_dtheta,
  const Matrix3d& cov_a_b,
  const Matrix3d& cov_w_b)
{
  assert(var_acc > 0.);
  assert(var_gyro > 0.);
  assert(var_acc_bias > 0.);
  assert(var_gyro_bias > 0.);
  assert(grav_W.z() < 0.);
  assert(eigen_tools::isPositive(cov_pos));
  assert(eigen_tools::isPositive(cov_vel));
  assert(eigen_tools::isPositive(cov_dtheta));
  assert(eigen_tools::isPositive(cov_a_b));
  assert(eigen_tools::isPositive(cov_w_b));

  var_acc_ = var_acc;
  var_gyro_ = var_gyro;
  var_acc_bias_ = var_acc_bias;
  var_gyro_bias_ = var_gyro_bias;
  grav_W_ = grav_W;
  mag_W_ = mag_W;

  // ノミナル状態を初期化
  nominal_state_.setZero();
  nominal_state_.block<3, 1>(POS_IDX, 0) = init_pos;
  nominal_state_.block<3, 1>(VEL_IDX, 0) = init_vel;
  nominal_state_.block<4, 1>(QUAT_IDX, 0) = quatToHamilton(init_quat).normalized();

  // 共分散行列を初期化
  P_.setZero();
  P_.block<3, 3>(dPOS_IDX, dPOS_IDX) = cov_pos;
  P_.block<3, 3>(dVEL_IDX, dVEL_IDX) = cov_vel;
  P_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = cov_dtheta;
  P_.block<3, 3>(dAB_IDX, dAB_IDX) = cov_a_b;
  P_.block<3, 3>(dWB_IDX, dWB_IDX) = cov_w_b;

  // (270) ヤコビアンの不変部分を埋める
  F_x_.setZero();
  F_x_.block<3, 3>(dPOS_IDX, dPOS_IDX).diagonal().fill(1.);
  F_x_.block<3, 3>(dVEL_IDX, dVEL_IDX).diagonal().fill(1.);
  F_x_.block<3, 3>(dAB_IDX, dAB_IDX).diagonal().fill(1.);
  F_x_.block<3, 3>(dWB_IDX, dWB_IDX).diagonal().fill(1.);
}

Matrix3d ErrorStateKalmanFilter::getDCM()
{
  return getQuaternion().matrix();
}

Quaterniond ErrorStateKalmanFilter::quatFromHamilton(const Vector4d& qHam)
{
  return Quaterniond((Vector4d() << qHam.block<3, 1>(1, 0), qHam.block<1, 1>(0, 0)).finished());
}

Vector4d ErrorStateKalmanFilter::quatToHamilton(const Quaterniond& q)
{
  return (Vector4d() << q.coeffs().block<1, 1>(3, 0), q.coeffs().block<3, 1>(0, 0)).finished();
}

Matrix3d ErrorStateKalmanFilter::getSkew(const Vector3d& in)
{
  Matrix3d out;
  out << 0, -in(2), in(1), in(2), 0, -in(0), -in(1), in(0), 0;
  return out;
}

Matrix3d ErrorStateKalmanFilter::rotVecToMat(const Vector3d& in)
{
  const double angle = in.norm();
  const Vector3d axis = (angle == 0) ? Vector3d(1, 0, 0) : in.normalized();
  AngleAxisd angle_axis(angle, axis);
  return angle_axis.toRotationMatrix();
}

Quaterniond ErrorStateKalmanFilter::rotVecToQuat(const Vector3d& in)
{
  const double angle = in.norm();
  Vector3d axis = (angle == 0) ? Vector3d(1, 0, 0) : in.normalized();
  return Quaterniond(AngleAxisd(angle, axis));
}

Vector3d ErrorStateKalmanFilter::quatToRotVec(const Quaterniond& q)
{
  AngleAxisd angle_axis(q);
  return angle_axis.angle() * angle_axis.axis();
}

void ErrorStateKalmanFilter::predictIMU(const Vector3d& a_m, const Vector3d& w_m, const double dt)
{
  assert(dt > 0.);

  const Matrix3d Rot = getDCM();
  const Vector3d acc_B = a_m - getAccelBias();
  const Vector3d acc_W = Rot * acc_B;
  const Vector3d delta_theta = (w_m - getGyroBias()) * dt;
  const Quaterniond q_delta_theta = rotVecToQuat(delta_theta);
  const Matrix3d R_delta_theta = q_delta_theta.toRotationMatrix();

  // (260) ノミナル状態のキネマティクス
  nominal_state_.block<3, 1>(POS_IDX, 0) += getVelocity() * dt + 0.5 * (acc_W + grav_W_) * SQ(dt);
  nominal_state_.block<3, 1>(VEL_IDX, 0) += (acc_W + grav_W_) * dt;
  nominal_state_.block<4, 1>(QUAT_IDX, 0) =
    quatToHamilton(getQuaternion() * q_delta_theta).normalized();

  // (270) ヤコビアンの可変部を更新
  F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX).diagonal().fill(dt);
  F_x_.block<3, 3>(dVEL_IDX, dTHETA_IDX) = -Rot * getSkew(acc_B) * dt;
  F_x_.block<3, 3>(dVEL_IDX, dAB_IDX) = -Rot * dt;
  F_x_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = R_delta_theta.transpose();
  F_x_.block<3, 3>(dTHETA_IDX, dWB_IDX).diagonal().fill(-dt);

  // (269): 共分散行列の予測値を更新
  P_ = F_x_ * P_ * F_x_.transpose();

  // 無理やり対称性を保存 (これが必須)
  eigen_tools::symmetrise(P_);

  // Inject process noise
  P_.diagonal().block<3, 1>(dVEL_IDX, 0).array() += var_acc_ * SQ(dt);
  P_.diagonal().block<3, 1>(dTHETA_IDX, 0).array() += var_gyro_ * SQ(dt);
  P_.diagonal().block<3, 1>(dAB_IDX, 0).array() += var_acc_bias_ * dt;
  P_.diagonal().block<3, 1>(dWB_IDX, 0).array() += var_gyro_bias_ * dt;

  // For debug
  // cout << "F_x:" << endl << F_x_ << endl;
}

Matrix<double, 4, 3> ErrorStateKalmanFilter::getQ_dtheta()
{
  Vector4d qby2 = 0.5 * getQuatVector();
  // Assing to letters for readability. Note Hamilton order.
  const double w = qby2[0];
  const double x = qby2[1];
  const double y = qby2[2];
  const double z = qby2[3];
  Matrix<double, 4, 3> Q_dtheta;
  Q_dtheta << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
  return Q_dtheta;
}

void ErrorStateKalmanFilter::measurePosition3D(const Vector3d& pos_meas, const Matrix3d& pos_cov)
{
  const Vector3d delta_pos = pos_meas - getPosition3D();

  Matrix<double, 3, dSTATE_SIZE> H;
  H.setZero();
  H.block<3, 3>(0, dPOS_IDX).diagonal().fill(1.);

  correct<3>(delta_pos, pos_cov, H);
}

void ErrorStateKalmanFilter::measurePosition2D(const Vector2d& xy_meas, const Matrix2d& xy_cov)
{
  const Vector2d delta_xy = xy_meas - getPosition2D();

  Matrix<double, 2, dSTATE_SIZE> H;
  H.setZero();
  H.block<2, 2>(0, dPOS_IDX).diagonal().fill(1.);

  correct<2>(delta_xy, xy_cov, H);
}

void ErrorStateKalmanFilter::measureAltitude(const double& z_meas, const double& z_cov)
{
  const double delta_z = z_meas - getAltitude();
  // std::cout << "Estimated altitude:" << endl << getAltitude() << endl;
  // std::cout << "Measured altitude:" << endl << z_meas << endl;

  Matrix<double, 1, dSTATE_SIZE> H;
  H.setZero();
  H(0, dALT_IDX) = 1.;

  correct<1>(Scalar(delta_z), Scalar(z_cov), H);
}

void ErrorStateKalmanFilter::measureVelocity(const Vector3d& vel_meas, const Matrix3d& vel_cov)
{
  const Vector3d delta_vel = vel_meas - getVelocity();
  // std::cout << "Estimated velocity:" << endl << getVelocity() << endl;
  // std::cout << "Measured velocity:" << endl << vel_meas << endl;

  Matrix<double, 3, dSTATE_SIZE> H;
  H.setZero();
  H.block<3, 3>(0, dVEL_IDX).diagonal().fill(1.);

  correct<3>(delta_vel, vel_cov, H);
}

void ErrorStateKalmanFilter::measureQuaternion(
  const Quaterniond& q_gb_meas,
  const Matrix3d& theta_cov)
{
  const Quaterniond q_gb_nominal = getQuaternion();
  const Quaterniond q_bNominal_bMeas = q_gb_nominal.conjugate() * q_gb_meas;
  const Vector3d delta_theta = quatToRotVec(q_bNominal_bMeas);

  // Because of the above construction, H is a trivial observation of dtheta
  Matrix<double, 3, dSTATE_SIZE> H;
  H.setZero();
  H.block<3, 3>(0, dTHETA_IDX).diagonal().fill(1.);

  // Apply update
  correct<3>(delta_theta, theta_cov, H);
}

void ErrorStateKalmanFilter::measureAcceleration(const Vector3d& acc_meas, const Matrix3d& acc_cov)
{
  const Matrix3d rot_B_W = getDCM().transpose();
  const Vector3d grav_B = rot_B_W * grav_W_;
  const Vector3d acc_nominal = -grav_B;
  const Vector3d delta_acc = acc_meas - acc_nominal;
  const Matrix3d grav_B_cross = getSkew(grav_B);

  // std::cout << "rot_B_W:" << endl << rot_B_W << endl;
  // std::cout << "grav_B:" << endl << grav_B << endl;
  // std::cout << "skew(grav_B):" << endl << grav_B_cross << endl;

  Matrix<double, 3, dSTATE_SIZE> H;
  H.setZero();
  // H.block(0, dTHETA_IDX, 3, 3) = -2 * getSkew(grav_B);  // これだとHが変更されない．なぜ？？
  H.block(0, dTHETA_IDX, 3, 3) = -2 * grav_B_cross;

  correct<3>(delta_acc, acc_cov, H);
}

void ErrorStateKalmanFilter::measureMagneticField(const Vector3d& mag_meas, const Matrix3d& mag_cov)
{
  const Matrix3d rot_B_W = getDCM().transpose();
  const Vector3d mag_B = rot_B_W * mag_W_;
  const Vector3d delta_mag = mag_meas - mag_B;
  const Matrix3d mag_B_cross = getSkew(mag_B);

  Matrix<double, 3, dSTATE_SIZE> H;
  H.setZero();
  H.block<3, 3>(0, dTHETA_IDX) = 2 * mag_B_cross;

  correct<3>(delta_mag, mag_cov, H);
}

void ErrorStateKalmanFilter::injectErrorState(const dStateVector& error_state)
{
  // (283) 観測した誤差をノミナル状態に反映
  const Vector3d dtheta = error_state.block<3, 1>(dTHETA_IDX, 0);
  const Quaterniond q_dtheta = rotVecToQuat(dtheta);
  nominal_state_.block<3, 1>(POS_IDX, 0) += error_state.block<3, 1>(dPOS_IDX, 0);
  nominal_state_.block<3, 1>(VEL_IDX, 0) += error_state.block<3, 1>(dVEL_IDX, 0);
  nominal_state_.block<4, 1>(QUAT_IDX, 0) = quatToHamilton(getQuaternion() * q_dtheta).normalized();
  nominal_state_.block<3, 1>(AB_IDX, 0) += error_state.block<3, 1>(dAB_IDX, 0);
  nominal_state_.block<3, 1>(WB_IDX, 0) += error_state.block<3, 1>(dWB_IDX, 0);

  // (286) ESKFを初期化 (不要)
  // FIXME: これをやるとバグる問題．symmetriseを挟むと若干マシになるがそれでもやらないほうがマシ．
  // const Matrix3d G_theta = I_3 - getSkew(0.5 * dtheta);
  // P_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) =
  //   G_theta * P_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) * G_theta.transpose();
  // eigen_tools::symmetrise(P_);
}
