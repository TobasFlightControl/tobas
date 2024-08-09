#include <eigen3/Eigen/LU>
#include <std_msgs/msg/float64.hpp>

#include <tobas_math/core.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_mr_thrust_estimation/thrust_estimator.hpp"

#define EPS 1e-6
#define GRAV_W Vector3d(0, 0, tobas_std::kGravity)

using namespace std;
using namespace Eigen;

namespace tobas_mr_thrust_estimation
{
ThrustEstimator::ThrustEstimator(const rclcpp::NodeOptions& options)
  : super(node, pnh, name), dynamics_(drone_), kf_(1),
{

  updateInternalDataStructures();

  kf_.initialize(Scalard(1), Scalard(kInitFactorStddev));
  kf_.setZero();

  factor_pub_ = createPublisher<std_msgs::msg::Float64>(tobas::kThrustCorrectionFactorTopic);

  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  rotor_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsTopic, &self::rotorSpeedsCb, this);


}

void ThrustEstimator::updateInternalDataStructures()
{
  dynamics_.updateInternalDataStructures();
}

void ThrustEstimator::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  if (odom->status != tobas_msgs::msg::Odometry::NO_ERROR)
    return;

  if (rotor_speeds_ == nullptr)
    return;

  const auto& R_W_B = odom->frame.M.data;
  const auto& acc_B = odom->accel.linear.data;
  const Vector3d grav_B = R_W_B.transpose() * GRAV_W;

  // 実際の推力に対するモデル推力の比率の観測値
  // 回転数が観測できない場合はロータのダイナミクスとの分離は不可能なため，全て推力定数の誤差に起因すると考える
  const auto real_thrust = dynamics_.mass() * (acc_B + grav_B).z();
  const auto model_thrust = dynamics_.thrustSum(rotor_speeds_->speeds);
  const auto factor_meas = max(model_thrust / real_thrust, kMinFactor);
  kf_.y(0) = factor_meas;

  // 観測ノイズの共分散
  // 実際は加速度ノイズとジャイロノイズの分散に比例する値のはずだが，
  // どうせプロセスノイズのスケールがわからないため観測ノイズのスケールも適当でよい．
  // 簡単のため最も影響の大きいと思われる加速度ノイズの分散をそのまま使う．
  kf_.R(0, 0) = odom->accel_covariance(2, 2) + EPS;

  // カルマンフィルタを更新
  kf_.update();

  // Publish estimated thrust correction factor
  const auto factor_msg =std::make_unique<std_msgs::msg::Float64>();
  factor_msg->data = kf_.state()(0);
  factor_pub_->publish(factor_msg);
}

void ThrustEstimator::rotorSpeedsCb(const tobas_msgs::msg::RotorSpeeds::ConstSharedPtr& rotor_speeds)
{
  rotor_speeds_ = rotor_speeds;
}

void ThrustEstimator::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  // プロセスノイズの分散
  // TODO: dtを反映
  kf_.Q(0, 0) = exp10(cfg.process_noise_variance_log10);

  TOBAS_INFO("New dynamic parameters are set.");
}
}  // namespace tobas_mr_thrust_estimation
