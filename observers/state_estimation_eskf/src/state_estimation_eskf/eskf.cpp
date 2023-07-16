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
  double acc_noise_density,
  double gyro_noise_density,
  double acc_random_walk,
  double gyro_random_walk,
  const Vector3d& grav_W,
  const Vector3d& mag_W,
  const Vector3d& init_pos,
  const Vector3d& init_vel,
  const Quaterniond& init_quat,
  const Matrix3d& init_pos_cov,
  const Matrix3d& init_vel_cov,
  const Matrix3d& init_dtheta_cov,
  const Matrix3d& init_acc_bias_cov,
  const Matrix3d& init_gyro_bias_cov)
{
  assert(acc_noise_density > 0.);
  assert(gyro_noise_density > 0.);
  assert(acc_random_walk > 0.);
  assert(gyro_random_walk > 0.);
  assert(grav_W.z() < 0.);
  assert(et::isSymmetric(init_pos_cov) && et::isSemiPositive(init_pos_cov));
  assert(et::isSymmetric(init_vel_cov) && et::isSemiPositive(init_vel_cov));
  assert(et::isSymmetric(init_dtheta_cov) && et::isSemiPositive(init_dtheta_cov));
  assert(et::isSymmetric(init_acc_bias_cov) && et::isSemiPositive(init_acc_bias_cov));
  assert(et::isSymmetric(init_gyro_bias_cov) && et::isSemiPositive(init_gyro_bias_cov));

  acc_noise_density_ = acc_noise_density;
  gyro_noise_density_ = gyro_noise_density;
  acc_random_walk_ = acc_random_walk;
  gyro_random_walk_ = gyro_random_walk;
  grav_W_ = grav_W;
  mag_W_ = mag_W;

  // ノミナル状態を初期化
  nominal_state_.setZero();
  nominal_state_.block<3, 1>(kPosIdx, 0) = init_pos;
  nominal_state_.block<3, 1>(kVelIdx, 0) = init_vel;
  nominal_state_.block<4, 1>(kQuatIdx, 0) = et::quaternionToHamilton(init_quat).normalized();

  // 共分散行列を初期化
  P_.setZero();
  P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx) = init_pos_cov;
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) = init_vel_cov;
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = init_dtheta_cov;
  P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx) = init_acc_bias_cov;
  P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx) = init_gyro_bias_cov;

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
  return et::hamiltonToQuaternion(getHamilton());
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

double ErrorStateKalmanFilter::getYaw() const
{
  const auto R_W_B = getDCM();
  return atan2(R_W_B(1, 0), R_W_B(0, 0));
}

Matrix3d ErrorStateKalmanFilter::getPositionCovariance() const
{
  return P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx);
}

Matrix3d ErrorStateKalmanFilter::getVelocityCovariance() const
{
  return P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx);
}

Matrix3d ErrorStateKalmanFilter::getOrientationCovariance() const
{
  return P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx);
}

Matrix3d ErrorStateKalmanFilter::getAccelBiasCovariance() const
{
  return P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx);
}

Matrix3d ErrorStateKalmanFilter::getGyroBiasCovariance() const
{
  return P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx);
}

void ErrorStateKalmanFilter::predictIMU(const Vector3d& a_m, const Vector3d& w_m, double dt)
{
  assert(dt > 0.);  // クオータニオンの正規化のためにdt = 0を許容できない
  assert(dt < kImuTimeGapThreshold);

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

  // 無理やり対称化 (これが必須)
  // プロセスノイズを加える前に対称化する必要がある
  et::symmetrise(P_);

  // Noise variance
  // FIXME: (264), (265)は論文と単位が異なるが，無理にdtの数で論文に単位を合わせると性能が劣化する
  const double sigma2_an = sqr(acc_noise_density_) / dt;   // (262) [m^2/s^4]
  const double sigma2_wn = sqr(gyro_noise_density_) / dt;  // (263) [rad^2/s^2]
  const double sigma2_aw = sqr(acc_random_walk_) / dt;     // (264) [m^2/s^6]
  const double sigma2_ww = sqr(gyro_random_walk_) / dt;    // (265) [rad^2/s^4]

  // Inject process noise
  P_.diagonal().block<3, 1>(kDeltaVelIdx, 0).array() += sigma2_an * sqr(dt);
  P_.diagonal().block<3, 1>(kDeltaThetaIdx, 0).array() += sigma2_wn * sqr(dt);
  P_.diagonal().block<3, 1>(kDeltaAccBiasIdx, 0).array() += sigma2_aw * dt;
  P_.diagonal().block<3, 1>(kDeltaGyroBiasIdx, 0).array() += sigma2_ww * dt;

  // NaN検出
  assert(et::isFinite(nominal_state_));
  assert(et::isFinite(F_x_));
  assert(et::isFinite(P_));
}

Matrix<double, 4, 3> ErrorStateKalmanFilter::getQ_dtheta()
{
  Vector4d qby2 = 0.5 * getHamilton();
  const double w = qby2(0);
  const double x = qby2(1);
  const double y = qby2(2);
  const double z = qby2(3);

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
  const Matrix3d grav_B_cross = et::crossMat(grav_B);  // 一時objectをblockに代入すると反映されない

  // std::cout << "rot_B_W:" << endl << rot_B_W << endl;
  // std::cout << "grav_B:" << endl << grav_B << endl;
  // std::cout << "skew(grav_B):" << endl << grav_B_cross << endl;

  Matrix<double, 3, kDeltaStateSize> H;
  H.setZero();
  H.block(0, kDeltaThetaIdx, 3, 3) = -2 * grav_B_cross;

  correct<3>(delta_acc, acc_cov, H);
}

void ErrorStateKalmanFilter::measureMagneticFieldRPY(
  const Vector3d& mag_meas,
  const Matrix3d& mag_cov)
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

void ErrorStateKalmanFilter::measureMagneticFieldYaw(
  double mag_meas_x,
  double mag_meas_y,
  double yaw_var)
{
  assert(yaw_var > 0.);

  // Compute innovation
  const double yaw_meas = wrapPi(atan2(mag_W_.y(), mag_W_.x()) - atan2(mag_meas_y, mag_meas_x));
  const double delta_yaw = wrapPi(yaw_meas - getYaw());

  // Choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
  constexpr double epsilon = 1e-6;
  const Quaterniond q = getQuaternion();

  bool can_use_A = false;
  const double SA0 = 2 * q.z();
  const double SA1 = 2 * q.y();
  const double SA2 = SA0 * q.w() + SA1 * q.x();
  const double SA3 = sqr(q.w()) + sqr(q.x()) - sqr(q.y()) - sqr(q.z());
  double SA4, SA5_inv;
  if (sqr(SA3) > epsilon)
  {
    SA4 = 1 / sqr(SA3);
    SA5_inv = sqr(SA2) * SA4 + 1;
    can_use_A = abs(SA5_inv) > epsilon;
  }

  bool can_use_B = false;
  const double SB0 = 2 * q.w();
  const double SB1 = 2 * q.x();
  const double SB2 = SB0 * q.z() + SB1 * q.y();
  const double SB4 = sqr(q.w()) + sqr(q.x()) - sqr(q.y()) - sqr(q.z());
  double SB3, SB5_inv;
  if (sqr(SB2) > epsilon)
  {
    SB3 = 1 / sqr(SB2);
    SB5_inv = SB3 * sqr(SB4) + 1;
    can_use_B = abs(SB5_inv) > epsilon;
  }

  // Compute output matrix
  RowVector4d H_yaw;
  if (can_use_A && (!can_use_B || abs(SA5_inv) >= abs(SB5_inv)))
  {
    const double SA5 = 1 / SA5_inv;
    const double SA6 = 1 / SA3;
    const double SA7 = SA2 * SA4;
    const double SA8 = 2 * SA7;
    const double SA9 = 2 * SA6;

    H_yaw(0) = SA5 * (SA0 * SA6 - SA8 * q.w());
    H_yaw(1) = SA5 * (SA1 * SA6 - SA8 * q.x());
    H_yaw(2) = SA5 * (SA1 * SA7 + SA9 * q.x());
    H_yaw(3) = SA5 * (SA0 * SA7 + SA9 * q.w());
  }
  else if (can_use_B && (!can_use_A || abs(SB5_inv) > abs(SA5_inv)))
  {
    const double SB5 = 1 / SB5_inv;
    const double SB6 = 1 / SB2;
    const double SB7 = SB3 * SB4;
    const double SB8 = 2 * SB7;
    const double SB9 = 2 * SB6;

    H_yaw(0) = -SB5 * (SB0 * SB6 - SB8 * q.z());
    H_yaw(1) = -SB5 * (SB1 * SB6 - SB8 * q.y());
    H_yaw(2) = -SB5 * (-SB1 * SB7 - SB9 * q.y());
    H_yaw(3) = -SB5 * (-SB0 * SB7 - SB9 * q.z());
  }
  else
  {
    return;
  }

  const auto Q_dtheta = getQ_dtheta();
  RowDeltaStateVector H;
  H.setZero();
  H.block<1, 3>(0, kDeltaThetaIdx) = H_yaw * Q_dtheta;

  // Update the quaternion states and covariance matrix
  correct<1>(Scalar(delta_yaw), Scalar(yaw_var), H);
}

void ErrorStateKalmanFilter::injectErrorState(const DeltaStateVector& error_state)
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
