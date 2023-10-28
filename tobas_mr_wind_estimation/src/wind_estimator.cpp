#include <Eigen/LU>

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Wind.h>

#include "../include/tobas_mr_wind_estimation/wind_estimator.hpp"

#define E_XY DiagonalMatrix<double, 3>(1, 1, 0)
#define GRAV_W Vector3d(0, 0, tobas::kGravity)

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_wind_estimation
{
WindEstimator::WindEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), dynamics_(drone_), kf_(kStateSize)
{
  getRosParams();
  drone_.loadFromParam(nh_);
  updateInternalDataStructures();

  kf_.initialize(Vector2d::Zero(), Vector2d::Constant(dh_std::sqr(kInitWindStddev)).asDiagonal());
  kf_.setZero();

  registerPublishers();
  registerSubscribers();
}

void WindEstimator::updateInternalDataStructures()
{
  dynamics_.updateInternalDataStructures();
}

void WindEstimator::getRosParams()
{
}

void WindEstimator::registerPublishers()
{
  wind_pub_ = nh_.advertise<tobas_msgs::Wind>(tobas::kWindTopic, 1);
}

void WindEstimator::registerSubscribers()
{
  pt_sub_ = nh_.subscribe(tobas::kPoseTwistTopic, 1, &self::poseTwistCb, this, tcpNoDelay());
  rotor_speeds_sub_ =
    nh_.subscribe(tobas::kRotorSpeedsTopic, 1, &self::rotorSpeedsCb, this, tcpNoDelay());
}

Matrix3d WindEstimator::velCoef(const Euler& R_W_B)
{
  const auto drag_rotor_sum = dynamics_.dragRotorSum(rotor_speeds_->speeds);
  const auto mass = dynamics_.mass();
  const Matrix3d R_B_W = R_W_B.toRotation().inverse().data;
  return (drag_rotor_sum / mass) * E_XY * R_B_W;
}

void WindEstimator::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void WindEstimator::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  if (!is_initialized_)
  {
    if (rotor_speeds_ != nullptr && pt->pose.pos.z() > tobas::kModelEstimationAltThr)
    {
      t_last_loop_ = pt->header.stamp;
      is_initialized_ = true;
      rosInfo(name_, "Start to estimate wind speed.");
    }

    // 風速推定器は制御器と相互依存しているため，準備ができるまでは風速0を発行する．
    auto wind_msg = boost::make_shared<tobas_msgs::Wind>();
    wind_msg->header.frame_id = tobas::kWorldFrame;
    wind_msg->header.stamp = pt->header.stamp;
    wind_msg->vel.data.setZero();
    wind_pub_.publish(wind_msg);

    return;
  }

  // 時刻を更新
  const auto dt = (pt->header.stamp - t_last_loop_).toSec();
  t_last_loop_ = pt->header.stamp;

  const Matrix3d R_W_B = pt->pose.euler.toRotation().data;
  const Vector3d vel_W = R_W_B * pt->twist.vel.data;
  const Vector3d& acc_B = pt->accel.linear.data;
  const Vector3d grav_B = R_W_B.transpose() * GRAV_W;

  // 速度から加速度への係数行列を計算
  // Cvのランクは2だから，
  // 1. 最小二乗法で3軸とも推定
  // 2. 風速の水平成分のみを推定
  // の2つの選択肢がある．
  // 1の場合は水平成分の大きな誤差を垂直成分で説明しようとしてしまい精度が落ちるため，2を採用している．
  const Matrix3d Cv = velCoef(pt->pose.euler);
  const Matrix2d Cv_hor_inv = Cv.topLeftCorner(kStateSize, kStateSize).inverse();  // 水平成分のみ

  // 風速の観測値
  const Vector2d wind_W_meas = Cv_hor_inv * (acc_B + grav_B + Cv * vel_W).head(kStateSize);
  kf_.y = wind_W_meas;

  // プロセスノイズの共分散
  const Vector2d relative_wind_vel = kf_.state() - pt->twist.vel.data.head(kStateSize);  // 相対風速
  dryden_.update(relative_wind_vel.norm(), pt->pose.pos.z(), dt);
  kf_.Q(0, 0) = dh_std::sqr(dryden_.noiseStddevLon());
  kf_.Q(1, 1) = dh_std::sqr(dryden_.noiseStddevLat());

  // 観測ノイズの共分散
  const auto vel_cov_B = Map<const Matrix3d>(pt->linear_velocity_covariance.data());
  const auto vel_cov_W = R_W_B * vel_cov_B * R_W_B.transpose();
  const auto hor_vel_cov_W = vel_cov_W.topLeftCorner(kStateSize, kStateSize);
  const auto acc_cov_B = Map<const Matrix3d>(pt->linear_acceleration_covariance.data());
  const auto hor_acc_cov_B = acc_cov_B.topLeftCorner(kStateSize, kStateSize);
  kf_.R = hor_vel_cov_W + Cv_hor_inv * hor_acc_cov_B * Cv_hor_inv.transpose();

  // カルマンフィルタを更新
  kf_.update();

  // Publish wind message
  auto wind_msg = boost::make_shared<tobas_msgs::Wind>();
  wind_msg->header.frame_id = tobas::kWorldFrame;
  wind_msg->header.stamp = pt->header.stamp;
  wind_msg->vel.data.head(kStateSize) = kf_.state();
  wind_pub_.publish(wind_msg);
}

void WindEstimator::rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds)
{
  rotor_speeds_ = rotor_speeds;
}
}  // namespace tobas_mr_wind_estimation
