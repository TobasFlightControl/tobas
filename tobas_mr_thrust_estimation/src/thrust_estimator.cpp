#include <Eigen/LU>
#include <std_msgs/Float64.h>

#include <dh_std_tools/math.hpp>
#include <dh_eigen_tools/typedef.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_mr_thrust_estimation/thrust_estimator.hpp"

#define GRAV_W Vector3d(0, 0, tobas::kGravity)

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_thrust_estimation
{
ThrustEstimator::ThrustEstimator(ros::NodeHandle nh, ros::NodeHandle pnh, const string& name)
  : super(nh, pnh, name), dynamics_(drone_), kf_(1)
{
  getRosParams();
  drone_.loadFromParam(nh_);
  updateInternalDataStructures();

  kf_.initialize(Scalard(1), Scalard(kInitFactorStddev));
  kf_.setZero();

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ThrustEstimator::updateInternalDataStructures()
{
  dynamics_.updateInternalDataStructures();
}

void ThrustEstimator::getRosParams()
{
}

void ThrustEstimator::registerPublishers()
{
  factor_pub_ = nh_.advertise<std_msgs::Float64>(tobas::kThrustCorrectionFactor, 1);
}

void ThrustEstimator::registerSubscribers()
{
  pt_sub_ = nh_.subscribe(tobas::kPoseTwistTopic, 1, &self::poseTwistCb, this, tcpNoDelay());
  rotor_speeds_sub_ =
    nh_.subscribe(tobas::kRotorSpeedsTopic, 1, &self::rotorSpeedsCb, this, tcpNoDelay());
}

void ThrustEstimator::eventCb(const tobas_msgs::EventConstPtr& event)
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

void ThrustEstimator::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  if (!is_initialized_)
  {
    if (rotor_speeds_ != nullptr && pt->pose.pos.z() > kAltitudeThreshold)
    {
      t_last_loop_ = pt->header.stamp;
      is_initialized_ = true;
      rosInfo(name_, "Start to estimate thrust correction factor.");
    }
  }

  // 時刻を更新
  const auto dt = (pt->header.stamp - t_last_loop_).toSec();
  t_last_loop_ = pt->header.stamp;

  const Matrix3d R_W_B = pt->pose.euler.toRotation().data;
  const Vector3d& acc_B = pt->accel.linear.data;
  const Vector3d grav_B = R_W_B.transpose() * GRAV_W;

  // 風速の観測値
  const auto model_thrust_sum = dynamics_.thrustSum(rotor_speeds_->speeds);
  const auto factor_meas = dynamics_.mass() * (acc_B + grav_B).z() / model_thrust_sum;
  kf_.y(0) = factor_meas;

  // 観測ノイズの共分散
  kf_.R(0, 0) = pt->linear_acceleration_covariance[8];

  // カルマンフィルタを更新
  kf_.update();

  // Publish estimated thrust correction factor
  auto factor_msg = boost::make_shared<std_msgs::Float64>();
  factor_msg->data = kf_.state()(0);
  factor_pub_.publish(factor_msg);
}

void ThrustEstimator::rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds)
{
  rotor_speeds_ = rotor_speeds;
}

void ThrustEstimator::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  // プロセスノイズの分散
  kf_.Q(0, 0) = cfg.process_noise_variance;
}
}  // namespace tobas_mr_thrust_estimation
