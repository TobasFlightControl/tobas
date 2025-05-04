#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_std_tools/stopwatch.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_eigen_tools/linalg.hpp>
#include <tobas_eigen_tools/geometry.hpp>

namespace eskf
{
/**
 * @brief 誤差状態カルマンフィルタ．
 *
 * 基本アルゴリズム: Quaternion kinematics for the error-state Kalman filter [Sola, 2017]
 * 日本語訳: https://www.flight.t.u-tokyo.ac.jp/?p=800
 *
 * 地磁気バイアス推定の拡張: Online 3-Axis Magnetometer Hard-Iron and Soft-Iron Bias and Angular Velocity Sensor Bias
 * Estimation Using Angular Velocity Sensors for Improved Dynamic Heading Accuracy [Spielvogel+, 2022]
 *
 * @note IMUフレームで考える．
 */
class ErrorStateKalmanFilter
{
  // ノミナル状態の添字
  static constexpr size_t kPosIdx = 0;
  static constexpr size_t kAltIdx = kPosIdx + 2;
  static constexpr size_t kVelIdx = kPosIdx + 3;
  static constexpr size_t kQuatIdx = kVelIdx + 3;
  static constexpr size_t kAccBiasIdx = kQuatIdx + 4;
  static constexpr size_t kGyroBiasIdx = kAccBiasIdx + 3;
  static constexpr size_t kMagHardBiasIdx = kGyroBiasIdx + 3;
  static constexpr size_t kMagSoftBiasIdx = kMagHardBiasIdx + 3;
  static constexpr size_t kGravIdx = kMagSoftBiasIdx + 6;
  static constexpr size_t kStateSize = kGravIdx + 1;

  // 誤差状態の添字
  static constexpr size_t kDeltaPosIdx = 0;
  static constexpr size_t kDeltaAltIdx = kDeltaPosIdx + 2;
  static constexpr size_t kDeltaVelIdx = kDeltaPosIdx + 3;
  static constexpr size_t kDeltaThetaIdx = kDeltaVelIdx + 3;
  static constexpr size_t kDeltaAccBiasIdx = kDeltaThetaIdx + 3;
  static constexpr size_t kDeltaGyroBiasIdx = kDeltaAccBiasIdx + 3;
  static constexpr size_t kDeltaMagHardBiasIdx = kDeltaGyroBiasIdx + 3;
  static constexpr size_t kDeltaMagSoftBiasIdx = kDeltaMagHardBiasIdx + 3;
  static constexpr size_t kDeltaGravIdx = kDeltaMagSoftBiasIdx + 6;
  static constexpr size_t kDeltaStateSize = kDeltaGravIdx + 1;

  // 変数の範囲
  static constexpr double kMaxAccBias = 1.;                 // [m/s^2]
  static constexpr double kMaxGyroBias = 0.1;               // [rad/s]
  static constexpr double kMaxMagHardBias = 2.;             // [-]
  static constexpr double kMinMagSoftBiasEigenValue = 0.1;  // [-]
  static constexpr double kMinGravity = 9.75;               // [m/s^2]
  static constexpr double kMaxGravity = 9.85;               // [m/s^2]

  // センサの不確かさの制限
  // 各センサの読みにはバイアスが乗っているため，静止時など極端に分散が小さくバイアス成分が優勢だと思われる場合は現実の不確かさを反映していない．
  // バイアスが乗った値を確かな値として扱い共分散が成長しないと，観測の補正が入らず姿勢が発散する恐れがあるため，その固有値に下限を与える．
  static constexpr double kMinGyroStddev = 0.01;  // [rad/s]
  static constexpr double kMinAccStddev = 0.1;    // [m/s^2]

  // その他
  static constexpr auto kStateHistoryTimeWindow = std::chrono::milliseconds(500);
  static constexpr double kDoMeasGravMinGValue = 0.1;  // [G]
  static constexpr double kDoMeasGravMaxGValue = 2.;   // [G]

  using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;
  using StateVector = Eigen::Vector<double, kStateSize>;
  using RowStateVector = Eigen::RowVector<double, kStateSize>;
  using DeltaStateMatrix = Eigen::Matrix<double, kDeltaStateSize, kDeltaStateSize>;
  using DeltaStateVector = Eigen::Vector<double, kDeltaStateSize>;
  using RowDeltaStateVector = Eigen::RowVector<double, kDeltaStateSize>;

public:
  explicit ErrorStateKalmanFilter();

  bool initialize(
    const Eigen::Vector3d& init_pos,
    const Eigen::Matrix3d& init_pos_cov,
    const Eigen::Vector3d& init_vel,
    const Eigen::Matrix3d& init_vel_cov,
    const Eigen::Quaterniond& init_quat,
    const Eigen::Matrix3d& init_dtheta_cov,
    const Eigen::Vector3d& init_acc_bias,
    const Eigen::Matrix3d& init_acc_bias_cov,
    const Eigen::Vector3d& init_gyro_bias,
    const Eigen::Matrix3d& init_gyro_bias_cov,
    const Eigen::Vector3d& init_mag_hard_bias,
    const Eigen::Matrix3d& init_mag_hard_bias_cov,
    const Eigen::Matrix3d& init_mag_soft_bias,
    const Eigen::Matrix6d& init_mag_soft_bias_cov,
    const double& init_grav,
    const double& init_grav_var,
    const std::chrono::steady_clock::time_point& time);

  bool initializePosition(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov);
  bool initializeVelocity(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov);
  bool initializeQuaternion(const Eigen::Quaterniond& value, const Eigen::Matrix3d& cov);
  bool initializeAccelBias(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov);
  bool initializeGyroBias(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov);
  bool initializeMagHardBias(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov);
  bool initializeMagSoftBias(const Eigen::Matrix3d& value, const Eigen::Matrix6d& cov);
  bool initializeGravity(const double& value, const double& var);

  void enableSecondIntegral(bool enable);
  void enableCovSymmetrisation(bool enable);
  void enableCovInitialization(bool enable);
  void enableJosephForm(bool enable);

  bool setAccBiasProcNoiseDensity(double value);
  bool setGyroBiasProcNoiseDensity(double value);
  bool setMagHardBiasProcNoiseDensity(double value);
  bool setMagSoftBiasProcNoiseDensity(double value);
  bool setGravProcNoiseDensity(double value);

  bool setMagneticFieldRef(const Eigen::Vector3d& mag_W);

  // Direct value getters
  inline Eigen::Vector3d getPosition() const;
  inline Eigen::Vector3d getVelocity() const;
  inline Eigen::Vector4d getHamilton() const;
  inline Eigen::Vector3d getAccelBias() const;
  inline Eigen::Vector3d getGyroBias() const;
  inline Eigen::Vector3d getMagHardBias() const;
  inline Eigen::Matrix3d getMagSoftBias() const;
  inline double getGravity() const;

  // Extended value getters
  inline Eigen::Quaterniond getQuaternion() const;

  inline Eigen::Matrix3d getPositionCovariance() const;
  inline Eigen::Matrix3d getVelocityCovariance() const;
  inline Eigen::Matrix3d getRotationCovariance() const;
  inline Eigen::Matrix3d getAccelBiasCovariance() const;
  inline Eigen::Matrix3d getGyroBiasCovariance() const;
  inline Eigen::Matrix3d getMagHardBiasCovariance() const;
  inline Eigen::Matrix6d getMagSoftBiasCovariance() const;
  inline double getGravityVariance() const;

  /**
   * @brief 加速度とジャイロから次の状態を予測し，姿勢を補正する．
   *
   * @param acc_meas [m/s^2] 加速度の観測値
   * @param gyro_meas [rad/s] ジャイロの観測値
   * @param acc_cov [m^2/s^4] 加速度の観測ノイズの共分散
   * @param gyro_cov [rad^2/s^2] ジャイロの観測ノイズの共分散
   * @param grav_cov [m^2/s^4] 重力加速度の観測ノイズの共分散
   * @param time [s] 現在時刻
   */
  double measureIMU(
    const Eigen::Vector3d& acc_meas,
    const Eigen::Vector3d& gyro_meas,
    const Eigen::Matrix3d& acc_cov,
    const Eigen::Matrix3d& gyro_cov,
    const Eigen::Matrix3d& grav_cov,
    const std::chrono::steady_clock::time_point& time);

  /**
   * @brief 位置の観測をノミナル状態に反映させる．
   *
   * @param pos_meas 世界座標系で表現された位置の観測値
   * @param pos_cov 位置の観測ノイズの共分散
   * @param offset IMUフレームで表現された，IMUフレームに対する観測フレームのオフセット
   *
   * @return Anomaly score
   */
  double measurePosition(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Matrix3d& pos_cov,
    const Eigen::Vector3d& offset,
    const std::chrono::steady_clock::time_point& time);
  double measureXY(
    const Eigen::Vector2d& xy_meas,
    const Eigen::Matrix2d& xy_cov,
    const std::chrono::steady_clock::time_point& time);
  double measureAltitude(const double& z_meas, const double& z_var, const std::chrono::steady_clock::time_point& time);
  /**
   * @brief 速度の観測をノミナル状態に反映させる．
   *
   * @param pos_meas 世界座標系で表現された速度の観測値
   * @param pos_cov 速度の観測ノイズの共分散
   * @param offset IMUフレームで表現された，IMUフレームに対する観測フレームのオフセット
   * @param gyro_meas ジャイロセンサの読み
   *
   * @return Anomaly score
   */
  double measureVelocity(
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix3d& vel_cov,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& gyro_meas,
    const std::chrono::steady_clock::time_point& time);
  double measurePosVel(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Matrix3d& pos_cov,
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix3d& vel_cov,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& gyro_meas,
    const std::chrono::steady_clock::time_point& time);
  double measureQuaternion(
    const Eigen::Quaterniond& q_meas,
    const Eigen::Matrix3d& theta_cov,
    const std::chrono::steady_clock::time_point& time);

  double measureMagneticField3d(
    const Eigen::Vector3d& mag_meas,
    const Eigen::Matrix3d& mag_cov,
    const std::chrono::steady_clock::time_point& time);

  double measureMagneticFieldYaw(
    const Eigen::Vector3d& mag_meas,
    const Eigen::Matrix3d& mag_cov,
    const std::chrono::steady_clock::time_point& time);

private:
  // Configuration
  bool enable_second_integral_ = false;
  bool enable_cov_symmetrisation_ = false;
  bool enable_cov_initialization_ = false;
  bool enable_joseph_form_ = true;
  double acc_bias_proc_noise_density_ = 0.;   // [m/s^2/√Hz] 加速度バイアスのプロセスノイズ密度
  double gyro_bias_proc_noise_density_ = 0.;  // [rad/s/√Hz] ジャイロバイアスのプロセスノイズ密度
  double mag_hard_bias_proc_noise_density_ = 0.;  // [/√Hz] 地磁気ハードアイアンバイアスのプロセスノイズ密度
  double mag_soft_bias_proc_noise_density_ = 0.;  // [/√Hz] 地磁気ソフトアイアンバイアスのプ密度ノイズ密度
  double grav_proc_noise_density_ = 0.;  // [m/s^2/√Hz] 重力加速度のプロセスノイズ密度

  StateVector x_;         // State vector of the filter
  DeltaStateMatrix P_;    // Covariance of the error state
  DeltaStateMatrix F_x_;  // Jacobian of the state transition
  DeltaStateMatrix G_;    // Jacobian of the error initialization

  // 出力行列
  Eigen::Matrix<double, 3, kDeltaStateSize> H_pos_;
  Eigen::Matrix<double, 2, kDeltaStateSize> H_xy_;
  Eigen::Matrix<double, 1, kDeltaStateSize> H_z_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_vel_;
  Eigen::Matrix<double, 6, kDeltaStateSize> H_pv_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_theta_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_mag_;
  Eigen::Matrix<double, 1, kDeltaStateSize> H_yaw_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_grav_;

  std::chrono::steady_clock::time_point t_last_imu_;
  tobas_std::TimestampedBuffer<StateVector> x_history_;
  Eigen::Vector3d mag_W_ = Eigen::Vector3d::Zero();

  tobas_std::Stopwatch stopwatch_;

  // Direct value getters
  inline Eigen::Vector3d getPosition(const StateVector& x) const;
  inline Eigen::Vector3d getVelocity(const StateVector& x) const;
  inline Eigen::Vector4d getHamilton(const StateVector& x) const;
  inline Eigen::Vector3d getAccelBias(const StateVector& x) const;
  inline Eigen::Vector3d getGyroBias(const StateVector& x) const;
  inline Eigen::Vector3d getMagHardBias(const StateVector& x) const;
  inline Eigen::Matrix3d getMagSoftBias(const StateVector& x) const;
  inline double getGravity(const StateVector& x) const;

  // Extended value getters
  inline Eigen::Vector2d getXY(const StateVector& x) const;
  inline double getAltitude(const StateVector& x) const;
  inline Eigen::Vector3d getPosition(const StateVector& x, const Eigen::Vector3d& offset) const;
  inline Eigen::Vector3d
  getVelocity(const StateVector& x, const Eigen::Vector3d& offset, const Eigen::Vector3d& gyro_meas) const;
  inline Eigen::Quaterniond getQuaternion(const StateVector& x) const;
  inline Eigen::Matrix3d getDCM(const StateVector& x) const;
  inline Eigen::Vector3d getEuler(const StateVector& x) const;
  inline Eigen::Vector3d getGravVector(const StateVector& x) const;

  /* (281) */
  Eigen::Matrix<double, 4, 3> getQ_dtheta(const StateVector& x) const;

  /* ベクトルvのqによる回転をqで偏微分したもの．d(q * v * q') / d(q)． */
  Eigen::Matrix<double, 3, 4> quatRotationDerivative(const StateVector& x, const Eigen::Vector3d& a) const;

  /* クオータニオンからヨーへの出力方程式． */
  Eigen::RowVector4d hamiltonToYawOutputMatrix(const StateVector& x) const;

  void setMagSoftBiasFromMatrix(const Eigen::Matrix3d& T);
  void applyConstraints();
  void resetStateHistory();

  /**
   * @brief 重力方向の観測．姿勢の修正に用いる．
   * https://www.dropbox.com/s/ijfnlkvcep1w0f2/%E5%A7%BF%E5%8B%A2%E6%8E%A8%E5%AE%9A%E3%81%AE%E5%9F%BA%E7%A4%8E.pdf
   *
   * @param acc_meas 加速度センサの読み．
   * @param grav_cov 観測による修正量を決めるパラメータ．
   * 数式的には共分散として扱うが，センサノイズに加えて推定姿勢の分散も影響するため一般に正しい値は分からないから調整すべき．
   *
   * @return Anomaly score
   */
  double measureGravity(
    const Eigen::Vector3d& acc_meas,
    const Eigen::Matrix3d& grav_cov,
    const std::chrono::steady_clock::time_point& time);

  /**
   * @brief 観測から状態と共分散の事後推定を求める
   *
   * @tparam M 観測の次元
   * @param delta_meas 観測とノミナル状態の誤差
   * @param meas_cov 観測ノイズの共分散
   * @param H 観測方程式
   *
   * @return Anomaly score
   */
  template <int M>
  double correct(
    const Eigen::Matrix<double, M, 1>& delta_meas,
    const Eigen::Matrix<double, M, M>& meas_cov,
    const Eigen::Matrix<double, M, kDeltaStateSize>& H);
};

inline Eigen::Vector3d ErrorStateKalmanFilter::getPosition() const
{
  return getPosition(x_);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getVelocity() const
{
  return getVelocity(x_);
}

inline Eigen::Vector4d ErrorStateKalmanFilter::getHamilton() const
{
  return getHamilton(x_);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getAccelBias() const
{
  return getAccelBias(x_);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getGyroBias() const
{
  return getGyroBias(x_);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getMagHardBias() const
{
  return getMagHardBias(x_);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getMagSoftBias() const
{
  return getMagSoftBias(x_);
}

inline double ErrorStateKalmanFilter::getGravity() const
{
  return getGravity(x_);
}

inline Eigen::Quaterniond ErrorStateKalmanFilter::getQuaternion() const
{
  return getQuaternion(x_);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getPositionCovariance() const
{
  return P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getVelocityCovariance() const
{
  return P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getRotationCovariance() const
{
  return P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getAccelBiasCovariance() const
{
  return P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getGyroBiasCovariance() const
{
  return P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getMagHardBiasCovariance() const
{
  return P_.block<3, 3>(kDeltaMagHardBiasIdx, kDeltaMagHardBiasIdx);
}

inline Eigen::Matrix6d ErrorStateKalmanFilter::getMagSoftBiasCovariance() const
{
  return P_.block<6, 6>(kDeltaMagSoftBiasIdx, kDeltaMagSoftBiasIdx);
}

inline double ErrorStateKalmanFilter::getGravityVariance() const
{
  return P_(kDeltaGravIdx, kDeltaGravIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getPosition(const StateVector& x) const
{
  return x.segment<3>(kPosIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getVelocity(const StateVector& x) const
{
  return x.segment<3>(kVelIdx);
}

inline Eigen::Vector4d ErrorStateKalmanFilter::getHamilton(const StateVector& x) const
{
  return x.segment<4>(kQuatIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getAccelBias(const StateVector& x) const
{
  return x.segment<3>(kAccBiasIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getGyroBias(const StateVector& x) const
{
  return x.segment<3>(kGyroBiasIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getMagHardBias(const StateVector& x) const
{
  return x.segment<3>(kMagHardBiasIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getMagSoftBias(const StateVector& x) const
{
  const Eigen::Vector6d tp = x.segment<6>(kMagSoftBiasIdx);
  const auto& a = tp(0);
  const auto& b = tp(1);
  const auto& c = tp(2);
  const auto& d = tp(3);
  const auto& e = tp(4);
  const auto& f = tp(5);
  return (Eigen::Matrix3d() << a, b, c, b, d, e, c, e, f).finished();
}

inline double ErrorStateKalmanFilter::getGravity(const StateVector& x) const
{
  return x(kGravIdx);
}

inline Eigen::Vector2d ErrorStateKalmanFilter::getXY(const StateVector& x) const
{
  return x.block<2, 1>(kPosIdx, 0);
}

inline double ErrorStateKalmanFilter::getAltitude(const StateVector& x) const
{
  return x(kAltIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getPosition(const StateVector& x, const Eigen::Vector3d& offset) const
{
  return getPosition(x) + getQuaternion(x) * offset;
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getVelocity(
  const StateVector& x,
  const Eigen::Vector3d& offset,
  const Eigen::Vector3d& gyro_meas) const
{
  return getVelocity(x) + getQuaternion(x) * (gyro_meas - getGyroBias(x)).cross(offset);
}

inline Eigen::Quaterniond ErrorStateKalmanFilter::getQuaternion(const StateVector& x) const
{
  return eigen::quaternionFromHamilton(getHamilton(x));
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getDCM(const StateVector& x) const
{
  return getQuaternion(x).toRotationMatrix();
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getEuler(const StateVector& x) const
{
  Eigen::Vector3d rpy;
  const auto q = getQuaternion(x);
  tobas_std::eulerFromQuaternion(q.x(), q.y(), q.z(), q.w(), rpy.x(), rpy.y(), rpy.z());
  return rpy;
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getGravVector(const StateVector& x) const
{
  return Eigen::Vector3d(0, 0, -getGravity(x));
}

template <int M>
double ErrorStateKalmanFilter::correct(
  const Eigen::Matrix<double, M, 1>& delta_meas,
  const Eigen::Matrix<double, M, M>& meas_cov,
  const Eigen::Matrix<double, M, kDeltaStateSize>& H)
{
  assert(eigen::isSymmetricPositiveDefinite(meas_cov));

  // (274) Compute kalman gain
  const Eigen::Matrix<double, kDeltaStateSize, M> PHt = P_ * H.transpose();
  const Eigen::Matrix<double, M, M> Sigma_inv = (H * PHt + meas_cov).inverse();
  const Eigen::Matrix<double, kDeltaStateSize, M> K = PHt * Sigma_inv;

  // (275) Compute error state
  const DeltaStateVector delta_x = K * delta_meas;

  // (276) Update covariance matrix
  const DeltaStateMatrix I_KH = DeltaStateMatrix::Identity() - K * H;
  if (enable_joseph_form_) {
    // 対称正定が保持されやすい
    const auto P1 = I_KH * P_.selfadjointView<Eigen::Lower>() * I_KH.transpose();
    const auto P2 = K * meas_cov.template selfadjointView<Eigen::Lower>() * K.transpose();
    P_ = P1 + P2;
  }
  else {
    // 理論通りだが数値的に不安定
    P_ = I_KH * P_;
  }

  // (283) Update state
  const Eigen::Vector3d dtheta = delta_x.segment<3>(kDeltaThetaIdx);
  const Eigen::Quaterniond q_dtheta = eigen::quaternionFromAngleAxis(dtheta);
  x_.segment<3>(kPosIdx) += delta_x.segment<3>(kDeltaPosIdx);
  x_.segment<3>(kVelIdx) += delta_x.segment<3>(kDeltaVelIdx);
  x_.segment<4>(kQuatIdx) = eigen::hamiltonFromQuaternion(getQuaternion() * q_dtheta);
  x_.segment<3>(kAccBiasIdx) += delta_x.segment<3>(kDeltaAccBiasIdx);
  x_.segment<3>(kGyroBiasIdx) += delta_x.segment<3>(kDeltaGyroBiasIdx);
  x_.segment<3>(kMagHardBiasIdx) += delta_x.segment<3>(kDeltaMagHardBiasIdx);
  x_.segment<6>(kMagSoftBiasIdx) += delta_x.segment<6>(kDeltaMagSoftBiasIdx);
  x_(kGravIdx) += delta_x(kDeltaGravIdx);

  // (286) Initialize ESKF (Optional)
  if (enable_cov_initialization_) {
    G_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = Eigen::Diagonal3d(1, 1, 1) - eigen::skew(0.5 * dtheta);
    P_ = G_ * P_.selfadjointView<Eigen::Lower>() * G_.transpose();  // TODO: 必要な部分のみ計算
  }

  // Apply constraints to avoid numerical errors
  applyConstraints();

  // Compute anomaly score
  const double anomaly_score = (delta_meas.transpose() * Sigma_inv * delta_meas)(0) / M;
  return anomaly_score;
}
}  // namespace eskf
