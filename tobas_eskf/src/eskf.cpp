#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_eskf/eskf.hpp"

#define E3 Diagonal3d(1, 1, 1)

using namespace std;
using namespace chrono;
using namespace Eigen;

namespace eskf
{
ErrorStateKalmanFilter::ErrorStateKalmanFilter() : x_history_(kStateHistoryTimeWindow)
{
  // 観測方程式の固定部分を埋める
  H_pos_.setZero();
  H_xy_.setZero();
  H_z_.setZero();
  H_vel_.setZero();
  H_pv_.setZero();
  H_theta_.setZero();
  H_acc_.setZero();
  H_mag_.setZero();

  H_pos_.block<3, 3>(0, kDeltaPosIdx).diagonal().setOnes();
  H_xy_.block<2, 2>(0, kDeltaPosIdx).diagonal().setOnes();
  H_z_(0, kDeltaAltIdx) = 1;
  H_vel_.block<3, 3>(0, kDeltaVelIdx).diagonal().setOnes();
  H_pv_.block<3, 3>(0, kDeltaPosIdx).diagonal().setOnes();
  H_pv_.block<3, 3>(3, kDeltaVelIdx).diagonal().setOnes();
  H_theta_.block<3, 3>(0, kDeltaThetaIdx).diagonal().setOnes();  // 回転の誤差を3Dベクトルとして観測
  H_acc_.block<3, 3>(0, kAccBiasIdx).diagonal().setOnes();
}

void ErrorStateKalmanFilter::initialize(
  const Vector3d& init_pos,
  const Vector3d& init_vel,
  const Quaterniond& init_quat,
  const Matrix3d& init_pos_cov,
  const Matrix3d& init_vel_cov,
  const Matrix3d& init_dtheta_cov,
  const Matrix3d& init_acc_bias_cov,
  const Matrix3d& init_gyro_bias_cov,
  const double& init_grav_var,
  const steady_clock::time_point& time)
{
  assert(eigen::isSymmetricSemiPositiveDefinite(init_pos_cov));
  assert(eigen::isSymmetricSemiPositiveDefinite(init_vel_cov));
  assert(eigen::isSymmetricSemiPositiveDefinite(init_dtheta_cov));
  assert(eigen::isSymmetricSemiPositiveDefinite(init_acc_bias_cov));
  assert(eigen::isSymmetricSemiPositiveDefinite(init_gyro_bias_cov));
  assert(init_grav_var >= 0);

  // Initialize nominal state
  x_.setZero();
  x_.segment<3>(kPosIdx) = init_pos;
  x_.segment<3>(kVelIdx) = init_vel;
  x_.segment<4>(kQuatIdx) = eigen::quaternionToHamilton(init_quat).normalized();
  x_(kGravIdx) = tobas_std::kGravity;

  // Initialize covariance
  P_.setZero();
  P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx) = init_pos_cov;
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) = init_vel_cov;
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = init_dtheta_cov;
  P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx) = init_acc_bias_cov;
  P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx) = init_gyro_bias_cov;
  P_(kDeltaGravIdx, kDeltaGravIdx) = init_grav_var;

  // Fill the constant part of jacobians
  F_x_.setIdentity();
  G_.setIdentity();

  t_last_imu_ = time;
  x_history_.add(time, x_);
}

void ErrorStateKalmanFilter::enableJosephForm(bool enable)
{
  use_joseph_form_ = enable;
}

void ErrorStateKalmanFilter::enableCovInitialization(bool enable)
{
  do_cov_initialization_ = enable;
}

void ErrorStateKalmanFilter::measureIMU(
  const Vector3d& acc_meas,
  const Vector3d& gyro_meas,
  const Matrix3d& acc_cov,
  const Matrix3d& gyro_cov,
  const double& acc_bias_proc_noise_var,
  const double& gyro_bias_proc_noise_var,
  const double& grav_proc_noise_var,
  const double& grav_meas_noise_var,
  const steady_clock::time_point& time)
{
  assert(eigen::isSymmetricSemiPositiveDefinite(acc_cov));
  assert(eigen::isSymmetricSemiPositiveDefinite(gyro_cov));
  assert(acc_bias_proc_noise_var >= 0);
  assert(gyro_bias_proc_noise_var >= 0);
  assert(grav_proc_noise_var >= 0);
  assert(grav_meas_noise_var > 0);

  // サンプリングタイムを計算して時刻を更新
  const auto dt = duration<double>(time - t_last_imu_).count();
  t_last_imu_ = time;

  // クオータニオンの正規化のためにdt = 0を許容できない
  if (dt <= 0)
  {
    cerr << "IMU time gap must be positive: " << dt << " <= 0 [sec]" << endl;
    return;
  }

  const Matrix3d W_Rot_B = getDCM(x_);
  const Vector3d acc_B = acc_meas - getAccelBias(x_);
  const Vector3d acc_W = W_Rot_B * acc_B;
  const Vector3d delta_theta = (gyro_meas - getGyroBias(x_)) * dt;
  const Quaterniond q_delta_theta = eigen::angleAxisToQuaternion(delta_theta);
  const Matrix3d R_delta_theta = q_delta_theta.toRotationMatrix();

  // (260) ノミナル状態のキネマティクス
  // x_.segment<3>(kPosIdx) += getVelocity() * dt + 0.5 * (acc_W + getGravVector()) * math::sqr(dt);
  x_.segment<3>(kPosIdx) += getVelocity() * dt;  // 積分誤差が大きくなるため二階積分は考えない
  x_.segment<3>(kVelIdx) += (acc_W + getGravVector(x_)) * dt;
  x_.segment<4>(kQuatIdx) = eigen::quaternionToHamilton(getQuaternion(x_) * q_delta_theta).normalized();

  // (270) ヤコビアンの可変部を更新
  F_x_.block<3, 3>(kDeltaPosIdx, kDeltaVelIdx).diagonal().fill(dt);
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaThetaIdx) = -W_Rot_B * eigen::skew(acc_B) * dt;
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaAccBiasIdx) = -W_Rot_B * dt;
  F_x_(kDeltaVelIdx + 2, kDeltaGravIdx) = -dt;
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = R_delta_theta.transpose();
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaGyroBiasIdx).diagonal().fill(-dt);

  // (269)第一項: 共分散行列の予測値を更新
  P_ = F_x_ * P_ * F_x_.transpose();  // TODO: sympyを用いるなどして必要な部分のみ計算
  eigen::symmetrise(P_);              // 対称化 (これが必須)

  // (269)第二項: プロセスノイズを印加
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) += W_Rot_B * acc_cov * W_Rot_B.transpose() * math::sqr(dt);
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) += W_Rot_B * gyro_cov * W_Rot_B.transpose() * math::sqr(dt);
  P_.diagonal().segment<3>(kDeltaAccBiasIdx).array() += acc_bias_proc_noise_var;
  P_.diagonal().segment<3>(kDeltaGyroBiasIdx).array() += gyro_bias_proc_noise_var;
  P_(kDeltaGravIdx, kDeltaGravIdx) += grav_proc_noise_var;

  // 状態の履歴を保存
  x_history_.add(time, x_);

  // 重力方向の観測
  // 加速度と姿勢には等式関係 (= 出力方程式) があるため，カルマンフィルタ理論に則って補正を行う．
  measureGravity(acc_meas, Vector3d::Constant(grav_meas_noise_var).asDiagonal(), time);
}

double ErrorStateKalmanFilter::measurePosition(
  const Vector3d& pos_meas,
  const Matrix3d& pos_cov,
  const Vector3d& offset,
  const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Vector3d delta_pos = pos_meas - getPosition(x, offset);

  // 姿勢による偏微分
  const auto dqvq_dq = quatRotationDerivative(x, offset);
  const auto Q_dtheta = getQ_dtheta(x);
  H_pos_.block<3, 3>(0, kDeltaThetaIdx) = dqvq_dq * Q_dtheta;

  return correct(delta_pos, pos_cov, H_pos_);
}

double
ErrorStateKalmanFilter::measureXY(const Vector2d& xy_meas, const Matrix2d& xy_cov, const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Vector2d delta_xy = xy_meas - getXY(x);
  return correct(delta_xy, xy_cov, H_xy_);
}

double
ErrorStateKalmanFilter::measureAltitude(const double& z_meas, const double& z_var, const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const double delta_z = z_meas - getAltitude(x);
  return correct(Scalard(delta_z), Scalard(z_var), H_z_);
}

double ErrorStateKalmanFilter::measureVelocity(
  const Vector3d& vel_meas,
  const Matrix3d& vel_cov,
  const Vector3d& offset,
  const Vector3d& gyro_meas,
  const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Vector3d gyro_nominal = gyro_meas - getGyroBias(x);
  const Vector3d gyro_offset = gyro_nominal.cross(offset);
  const Vector3d vel_nominal = getVelocity(x) + getQuaternion(x) * gyro_offset;
  const Vector3d delta_vel = vel_meas - vel_nominal;

  // 姿勢による偏微分
  const auto dqvq_dq = quatRotationDerivative(x, gyro_offset);
  const auto Q_dtheta = getQ_dtheta(x);
  H_vel_.block<3, 3>(0, kDeltaThetaIdx) = dqvq_dq * Q_dtheta;

  // ジャイロバイアスによる偏微分
  H_vel_.block<3, 3>(0, kDeltaGyroBiasIdx) = getDCM(x) * eigen::skew(offset);

  return correct(delta_vel, vel_cov, H_vel_);
}

double ErrorStateKalmanFilter::measurePosVel(
  const Vector3d& pos_meas,
  const Matrix3d& pos_cov,
  const Vector3d& vel_meas,
  const Matrix3d& vel_cov,
  const Vector3d& offset,
  const Vector3d& gyro_meas,
  const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  // 観測誤差
  Vector6d delta;
  const Vector3d gyro_nominal = gyro_meas - getGyroBias(x);
  const Vector3d gyro_offset = gyro_nominal.cross(offset);
  const Vector3d vel_nominal = getVelocity(x) + getQuaternion(x) * gyro_offset;
  delta.head<3>() = pos_meas - getPosition(x, offset);  // 位置の誤差
  delta.tail<3>() = vel_meas - vel_nominal;             // 速度の誤差

  // 観測方程式
  const auto Q_dtheta = getQ_dtheta(x);
  const auto pos_q_deriv = quatRotationDerivative(x, offset);
  const auto vel_q_deriv = quatRotationDerivative(x, gyro_offset);
  H_pv_.block<3, 3>(0, kDeltaThetaIdx) = pos_q_deriv * Q_dtheta;  // 位置の姿勢による偏微分
  H_pv_.block<3, 3>(3, kDeltaThetaIdx) = vel_q_deriv * Q_dtheta;  // 速度の姿勢による偏微分
  H_pv_.block<3, 3>(0, kDeltaGyroBiasIdx) = getDCM(x) * eigen::skew(offset);

  // 共分散
  Matrix6d cov;
  cov.topLeftCorner<3, 3>() = pos_cov;
  cov.bottomRightCorner<3, 3>() = vel_cov;
  cov.topRightCorner<3, 3>().setZero();
  cov.bottomLeftCorner<3, 3>().setZero();

  // 事後推定を更新
  return correct(delta, cov, H_pv_);
}

double ErrorStateKalmanFilter::measureQuaternion(
  const Quaterniond& q_meas,
  const Matrix3d& theta_cov,
  const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Quaterniond q_nominal = getQuaternion(x);
  const Quaterniond q_error = q_nominal.conjugate() * q_meas;  // 回転の誤差
  const Vector3d delta_theta = eigen::quaternionToAngleAxis(q_error);

  return correct(delta_theta, theta_cov, H_theta_);
}

double ErrorStateKalmanFilter::measureMagneticField(
  const Vector3d& mag_meas,
  const Matrix3d& mag_cov,
  const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  // 地磁気をヨー角のみ機体と一致し，XY軸が地面と平行な地上座標系Gに移す．
  const auto R_W_B = getDCM(x);
  const auto yaw_pred = atan2(R_W_B(1, 0), R_W_B(0, 0));
  const AngleAxisd R_W_G(yaw_pred, Vector3d::UnitZ());
  const auto mag_G = R_W_G.inverse() * (R_W_B * mag_meas);  // 後ろから計算することで計算量を削減
  const auto mx = mag_G.x();
  const auto my = mag_G.y();

  // Compute innovation
  const auto yaw_ref = atan2(mag_ref_.y(), mag_ref_.x());
  const auto yaw_meas = yaw_ref - atan2(my, mx);
  const auto delta_yaw = algo::wrapPi(yaw_meas - yaw_pred);

  // 地磁気の分散からヨー角の分散を推定 (memo: 2-75)
  const auto mx_std = sqrt(mag_cov(0, 0));
  const auto my_std = sqrt(mag_cov(1, 1));
  const auto yaw_std = (fabs(mx) * my_std + fabs(my) * mx_std) / (math::sqr(mx) + math::sqr(my));
  const auto yaw_var = math::sqr(yaw_std);

  // Choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
  constexpr double kEpsilon = 1e-6;
  const Quaterniond q = getQuaternion(x);

  bool can_use_A = false;
  const double SA0 = 2 * q.z();
  const double SA1 = 2 * q.y();
  const double SA2 = SA0 * q.w() + SA1 * q.x();
  const double SA3 = math::sqr(q.w()) + math::sqr(q.x()) - math::sqr(q.y()) - math::sqr(q.z());
  double SA4, SA5_inv;
  if (math::sqr(SA3) > kEpsilon)
  {
    SA4 = 1 / math::sqr(SA3);
    SA5_inv = math::sqr(SA2) * SA4 + 1;
    can_use_A = fabs(SA5_inv) > kEpsilon;
  }

  bool can_use_B = false;
  const double SB0 = 2 * q.w();
  const double SB1 = 2 * q.x();
  const double SB2 = SB0 * q.z() + SB1 * q.y();
  const double SB4 = math::sqr(q.w()) + math::sqr(q.x()) - math::sqr(q.y()) - math::sqr(q.z());
  double SB3, SB5_inv;
  if (math::sqr(SB2) > kEpsilon)
  {
    SB3 = 1 / math::sqr(SB2);
    SB5_inv = SB3 * math::sqr(SB4) + 1;
    can_use_B = fabs(SB5_inv) > kEpsilon;
  }

  // Compute output matrix
  RowVector4d H_yaw;
  if (can_use_A && (!can_use_B || fabs(SA5_inv) >= fabs(SB5_inv)))
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
  else if (can_use_B && (!can_use_A || fabs(SB5_inv) > fabs(SA5_inv)))
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
    cerr << "Unable to compute the output matrix of yaw angle observation." << endl;
    return INFINITY;
  }

  const auto Q_dtheta = getQ_dtheta(x);
  H_mag_.block<1, 3>(0, kDeltaThetaIdx) = H_yaw * Q_dtheta;

  // Update the quaternion states and covariance matrix
  return correct(Scalard(delta_yaw), Scalard(yaw_var), H_mag_);
}

Matrix<double, 4, 3> ErrorStateKalmanFilter::getQ_dtheta(const StateVector& x) const
{
  const Vector4d qby2 = 0.5 * getHamilton(x);
  const double& qw = qby2(0);
  const double& qx = qby2(1);
  const double& qy = qby2(2);
  const double& qz = qby2(3);

  Matrix<double, 4, 3> Q_dtheta;
  Q_dtheta << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw;

  return Q_dtheta;
}

Matrix<double, 3, 4> ErrorStateKalmanFilter::quatRotationDerivative(const StateVector& x, const Vector3d& a) const
{
  const Vector4d ham = getHamilton(x);
  const double& w = ham(0);
  const Vector3d v = ham.tail<3>();

  Matrix<double, 3, 4> res;
  res.block<3, 1>(0, 0) = 2 * (w * a - a.cross(v));
  res.block<3, 3>(0, 1) = 2 * (a.dot(v) * E3 + v * a.transpose() - a * v.transpose() - w * eigen::skew(a));

  return res;
}

double ErrorStateKalmanFilter::measureGravity(
  const Vector3d& acc_meas,
  const Matrix3d& grav_cov,
  const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Matrix3d R_B_W = getDCM(x).transpose();
  const Vector3d grav_B = R_B_W * getGravVector(x);
  const Vector3d acc_ref = getAccelBias(x) - grav_B;  // 動的な加速度なしで観測されるべき加速度
  const Vector3d delta_acc = acc_meas - acc_ref;  // TODO: モデルから推定した動的加速度を引いた値を観測値とする

  H_acc_.block<3, 3>(0, kDeltaThetaIdx) = -2 * eigen::skew(grav_B);
  H_acc_.col(kDeltaGravIdx) = R_B_W.col(2);
  return correct(delta_acc, grav_cov, H_acc_);
}
}  // namespace eskf
