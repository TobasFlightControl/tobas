#include <Eigen/LU>
#include <std_msgs/Float64.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_mr_thrust_estimation/thrust_estimator.hpp"

#define EPS 1e-6
#define GRAV_W Vector3d(0, 0, tobas::kGravity)

using namespace std;
using namespace Eigen;
using namespace tobas_kdl;

namespace tobas_mr_thrust_estimation
{
ThrustEstimator::ThrustEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), dynamics_(drone_), kf_(1), server_(pnh_)
{
  drone_.loadFromParam(nh_);
  updateInternalDataStructures();

  kf_.initialize(Scalard(1), Scalard(kInitFactorStddev));
  kf_.setZero();

  factor_pub_ = nh_.advertise<std_msgs::Float64>(tobas::kThrustCorrectionFactorTopic, 1);

  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  rotor_speeds_sub_ = nh_.subscribe(tobas::kRotorSpeedsTopic, 1, &self::rotorSpeedsCb, this, tcpNoDelay());

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ThrustEstimator::updateInternalDataStructures()
{
  dynamics_.updateInternalDataStructures();
}

void ThrustEstimator::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  if (odom->status != tobas_msgs::Odometry::NO_ERROR)
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
  kf_.R(0, 0) = odom->linear_acceleration_covariance[8] + EPS;

  // カルマンフィルタを更新
  kf_.update();

  // Publish estimated thrust correction factor
  const auto factor_msg = boost::make_shared<std_msgs::Float64>();
  factor_msg->data = kf_.state()(0);
  factor_pub_.publish(factor_msg);
}

void ThrustEstimator::rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds)
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
