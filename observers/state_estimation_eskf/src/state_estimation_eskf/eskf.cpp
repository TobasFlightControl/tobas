#include <dh_std_tools/math.hpp>
#include <dh_eigen_tools/linalg.hpp>
#include <dh_eigen_tools/geometry.hpp>

#include "../../include/state_estimation_eskf/eskf.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;
namespace et = eigen_tools;

namespace state_estimation_eskf
{
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
  assert(et::isPositive(cov_pos));
  assert(et::isPositive(cov_vel));
  assert(et::isPositive(cov_dtheta));
  assert(et::isPositive(cov_a_b));
  assert(et::isPositive(cov_w_b));

  var_acc_ = var_acc;
  var_gyro_ = var_gyro;
  var_acc_bias_ = var_acc_bias;
  var_gyro_bias_ = var_gyro_bias;
  grav_W_ = grav_W;
  mag_W_ = mag_W;

  // ノミナル状態を初期化
  nominal_state_.setZero();
  nominal_state_.block<3, 1>(kPosIdx, 0) = init_pos;
  nominal_state_.block<3, 1>(kVelIdx, 0) = init_vel;
  nominal_state_.block<4, 1>(kQuatIdx, 0) = et::quaternionToHamilton(init_quat).normalized();

  // 共分散行列を初期化
  P_.setZero();
  P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx) = cov_pos;
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) = cov_vel;
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = cov_dtheta;
  P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx) = cov_a_b;
  P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx) = cov_w_b;

  // (270) ヤコビアンの不変部分を埋める
  F_x_.setZero();
  F_x_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx).diagonal().fill(1.);
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx).diagonal().fill(1.);
  F_x_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx).diagonal().fill(1.);
  F_x_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx).diagonal().fill(1.);
}

Vector3d ErrorStateKalmanFilter::getXYZ() const
{
  return nominal_state_.block<3, 1>(kPosIdx, 0);
}

Vector2d ErrorStateKalmanFilter::getXY() const
{
  return nominal_state_.block<2, 1>(kPosIdx, 0);
}

double ErrorStateKalmanFilter::getAltitude() const
{
  return nominal_state_(kAltIdx);
}

Vector3d ErrorStateKalmanFilter::getVelocity() const
{
  return nominal_state_.block<3, 1>(kVelIdx, 0);
}

Quaterniond ErrorStateKalmanFilter::getQuaternion() const
{
  return et::hamiltonToQuaternion(getQuatVector());
}

Vector3d ErrorStateKalmanFilter::getAccelBias() const
{
  return nominal_state_.block<3, 1>(kAccBiasIdx, 0);
}

Vector3d ErrorStateKalmanFilter::getGyroBias() const
{
  return nominal_state_.block<3, 1>(kGyroBiasIdx, 0);
}

Matrix3d ErrorStateKalmanFilter::getDCM() const
{
  return getQuaternion().toRotationMatrix();
}

void ErrorStateKalmanFilter::predictIMU(const Vector3d& a_m, const Vector3d& w_m, double dt)
{
  assert(dt > 0.);

  const Matrix3d Rot = getDCM();
  const Vector3d acc_B = a_m - getAccelBias();
  const Vector3d acc_W = Rot * acc_B;
  const Vector3d delta_theta = (w_m - getGyroBias()) * dt;
  const Quaterniond q_delta_theta = et::angleAxisToQuaternion(delta_theta);
  const Matrix3d R_delta_theta = q_delta_theta.toRotationMatrix();

  // (260) ノミナル状態のキネマティクス
  nominal_state_.block<3, 1>(kPosIdx, 0) += getVelocity() * dt + 0.5 * (acc_W + grav_W_) * sqr(dt);
  nominal_state_.block<3, 1>(kVelIdx, 0) += (acc_W + grav_W_) * dt;
  nominal_state_.block<4, 1>(kQuatIdx, 0) =
    et::quaternionToHamilton(getQuaternion() * q_delta_theta).normalized();

  // (270) ヤコビアンの可変部を更新
  F_x_.block<3, 3>(kDeltaPosIdx, kDeltaVelIdx).diagonal().fill(dt);
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaThetaIdx) = -Rot * et::crossMat(acc_B) * dt;
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaAccBiasIdx) = -Rot * dt;
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = R_delta_theta.transpose();
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaGyroBiasIdx).diagonal().fill(-dt);

  // (269): 共分散行列の予測値を更新
  P_ = F_x_ * P_ * F_x_.transpose();

  // 無理やり対称性を保存 (これが必須)
  et::symmetrise(P_);

  // Inject process noise
  P_.diagonal().block<3, 1>(kDeltaVelIdx, 0).array() += var_acc_ * sqr(dt);
  P_.diagonal().block<3, 1>(kDeltaThetaIdx, 0).array() += var_gyro_ * sqr(dt);
  P_.diagonal().block<3, 1>(kDeltaAccBiasIdx, 0).array() += var_acc_bias_ * dt;
  P_.diagonal().block<3, 1>(kDeltaGyroBiasIdx, 0).array() += var_gyro_bias_ * dt;

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

void ErrorStateKalmanFilter::measureXYZ(const Vector3d& pos_meas, const Matrix3d& pos_cov)
{
  const Vector3d delta_pos = pos_meas - getXYZ();

  Matrix<double, 3, kDeltaStateSize> H;
  H.setZero();
  H.block<3, 3>(0, kDeltaPosIdx).diagonal().fill(1.);

  correct<3>(delta_pos, pos_cov, H);
}

void ErrorStateKalmanFilter::measureXY(const Vector2d& xy_meas, const Matrix2d& xy_cov)
{
  const Vector2d delta_xy = xy_meas - getXY();

  Matrix<double, 2, kDeltaStateSize> H;
  H.setZero();
  H.block<2, 2>(0, kDeltaPosIdx).diagonal().fill(1.);

  correct<2>(delta_xy, xy_cov, H);
}

void ErrorStateKalmanFilter::measureAltitude(const double& z_meas, const double& z_cov)
{
  const double delta_z = z_meas - getAltitude();
  // std::cout << "Estimated altitude:" << endl << getAltitude() << endl;
  // std::cout << "Measured altitude:" << endl << z_meas << endl;

  Matrix<double, 1, kDeltaStateSize> H;
  H.setZero();
  H(0, kDeltaAltIdx) = 1.;

  correct<1>(Scalar(delta_z), Scalar(z_cov), H);
}

void ErrorStateKalmanFilter::measureVelocity(const Vector3d& vel_meas, const Matrix3d& vel_cov)
{
  const Vector3d delta_vel = vel_meas - getVelocity();
  // std::cout << "Estimated velocity:" << endl << getVelocity() << endl;
  // std::cout << "Measured velocity:" << endl << vel_meas << endl;

  Matrix<double, 3, kDeltaStateSize> H;
  H.setZero();
  H.block<3, 3>(0, kDeltaVelIdx).diagonal().fill(1.);

  correct<3>(delta_vel, vel_cov, H);
}

void ErrorStateKalmanFilter::measureQuaternion(const Quaterniond& q_meas, const Matrix3d& theta_cov)
{
  const Quaterniond q_nominal = getQuaternion();
  const Quaterniond q_nominal_meas = q_nominal.conjugate() * q_meas;  // 回転の誤差
  const Vector3d delta_theta = et::quaternionToAngleAxis(q_nominal_meas);

  // 回転の誤差を3次元ベクトルとして観測
  Matrix<double, 3, kDeltaStateSize> H;
  H.setZero();
  H.block<3, 3>(0, kDeltaThetaIdx).diagonal().fill(1.);

  // 事後推定を計算
  correct<3>(delta_theta, theta_cov, H);
}

void ErrorStateKalmanFilter::measureAcceleration(const Vector3d& acc_meas, const Matrix3d& acc_cov)
{
  const Matrix3d rot_B_W = getDCM().transpose();
  const Vector3d grav_B = rot_B_W * grav_W_;
  const Vector3d acc_nominal = -grav_B;
  const Vector3d delta_acc = acc_meas - acc_nominal;
  const Matrix3d grav_B_cross = et::crossMat(grav_B);

  // std::cout << "rot_B_W:" << endl << rot_B_W << endl;
  // std::cout << "grav_B:" << endl << grav_B << endl;
  // std::cout << "skew(grav_B):" << endl << grav_B_cross << endl;

  Matrix<double, 3, kDeltaStateSize> H;
  H.setZero();
  // H.block(0, kDeltaThetaIdx, 3, 3) = -2 * et::crossMat(grav_B);  //
  // これだとHが変更されない．なぜ？？
  H.block(0, kDeltaThetaIdx, 3, 3) = -2 * grav_B_cross;

  correct<3>(delta_acc, acc_cov, H);
}

void ErrorStateKalmanFilter::measureMagneticField(const Vector3d& mag_meas, const Matrix3d& mag_cov)
{
  const Matrix3d rot_B_W = getDCM().transpose();
  const Vector3d mag_B = rot_B_W * mag_W_;
  const Vector3d delta_mag = mag_meas - mag_B;
  const Matrix3d mag_B_cross = et::crossMat(mag_B);

  Matrix<double, 3, kDeltaStateSize> H;
  H.setZero();
  H.block<3, 3>(0, kDeltaThetaIdx) = 2 * mag_B_cross;

  correct<3>(delta_mag, mag_cov, H);
}

void ErrorStateKalmanFilter::injectErrorState(const dStateVector& error_state)
{
  // (283) 観測した誤差をノミナル状態に反映
  const Vector3d dtheta = error_state.block<3, 1>(kDeltaThetaIdx, 0);
  const Quaterniond q_dtheta = et::angleAxisToQuaternion(dtheta);
  nominal_state_.block<3, 1>(kPosIdx, 0) += error_state.block<3, 1>(kDeltaPosIdx, 0);
  nominal_state_.block<3, 1>(kVelIdx, 0) += error_state.block<3, 1>(kDeltaVelIdx, 0);
  nominal_state_.block<4, 1>(kQuatIdx, 0) =
    et::quaternionToHamilton(getQuaternion() * q_dtheta).normalized();
  nominal_state_.block<3, 1>(kAccBiasIdx, 0) += error_state.block<3, 1>(kDeltaAccBiasIdx, 0);
  nominal_state_.block<3, 1>(kGyroBiasIdx, 0) += error_state.block<3, 1>(kDeltaGyroBiasIdx, 0);

  // (286) ESKFを初期化 (不要)
  // FIXME: これをやるとバグる問題．symmetriseを挟むと若干マシになるがそれでもやらないほうがマシ．
  // const Matrix3d G_theta = I3 - et::crossMat(0.5 * dtheta);
  // P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) =
  //   G_theta * P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) * G_theta.transpose();
  // et::symmetrise(P_);
}
}  // namespace state_estimation_eskf
