
#include <tobas_math/core.hpp>
#include <tobas_algorithm/kahan.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Imu.h>

#include "../include/tobas_navio_ros/imu_handler.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_navio_ros
{
ImuHandler::ImuHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), property_client_(nh_, kPropertyServerFC)
{
  PRINT_DEBUG("ImuHandler::ImuHandler");

  if (!imu_.probe())
    TOBAS_EXIT("IMU not enabled.");
  imu_.initialize();

  reloadConfig();

  imu_pub_ = nh_.advertise<tobas_msgs::Imu>(tobas::kImuTopic, 1);
  reload_config_srv_ = nh_.advertiseService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);

  // コンストラクタで時間をとると他のNodeletがスタックするため，タイマーコールバックで初期化する．
  initialize_timer_ = nh_.createTimer(ros::Duration(0), &self::initializeTimerCb, this, true);

  // メインタイマーはジャイロのバイアスが測定してからスタートする
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this, false, false);

  PRINT_DEBUG("/ImuHandler::ImuHandler");
}

bool ImuHandler::reloadConfig()
{
  if (property_client_.get(kConfigKey_AccOffsetX, acc_bias_.x()) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    acc_bias_.setZero();
    return false;
  }
  if (property_client_.get(kConfigKey_AccOffsetY, acc_bias_.y()) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    acc_bias_.setZero();
    return false;
  }
  if (property_client_.get(kConfigKey_AccOffsetZ, acc_bias_.z()) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    acc_bias_.setZero();
    return false;
  }

  return true;
}

bool ImuHandler::reloadConfigCb(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
{
  if (!reloadConfig())
  {
    res.success = false;
    res.message = "Failed to reload configurations.";
    return true;
  }

  res.success = true;
  return true;
}

void ImuHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Update IMU
  imu_.updateAccelerometer();
  imu_.updateGyroscope();

  // Read IMU
  imu_.readAccelerometer(&acc_.x(), &acc_.y(), &acc_.z());
  imu_.readGyroscope(&gyro_.x(), &gyro_.y(), &gyro_.z());

  // Update noise filters
  const auto dt = (event.current_real - event.last_real).toSec();
  for (size_t i = 0; i < 3; ++i)
  {
    acc_noise_[i].update(acc_(i), dt);
    gyro_noise_[i].update(gyro_(i), dt);
  }

  // Create messages
  const auto imu_msg = boost::make_shared<tobas_msgs::Imu>();

  // Fill headers
  imu_msg->header.stamp = event.current_real;

  // Fill data (Convert to NWU coordinate system)
  const Vector3f acc = acc_ - acc_bias_;
  imu_msg->accel.x(acc.y());
  imu_msg->accel.y(-acc.x());
  imu_msg->accel.z(acc.z());

  const Vector3f gyro = gyro_ - gyro_bias_;
  imu_msg->gyro.x(gyro.y());
  imu_msg->gyro.y(-gyro.x());
  imu_msg->gyro.z(gyro.z());

  // Fill covariance matrices
  imu_msg->accel_covariance.setZero();
  imu_msg->gyro_covariance.setZero();
  for (size_t i = 0; i < 3; ++i)
  {
    imu_msg->accel_covariance(i, i) = acc_noise_[i].noiseVariance();
    imu_msg->gyro_covariance(i, i) = gyro_noise_[i].noiseVariance();
  }

  // Publish messages
  imu_pub_.publish(imu_msg);
}

void ImuHandler::initializeTimerCb(const ros::TimerEvent&)
{
  measureGyroBias();
  initializeNoiseFilters();

  main_timer_.start();
}

void ImuHandler::measureGyroBias()
{
  // 角速度の和を取得
  array<algo::Kahan<float>, 3> gyro_sum;

  ros::Rate rate(kSamplingRate);
  for (int i = 0; i < kMeasureGyroBiasCount; ++i)
  {
    // 角速度を取得
    imu_.updateGyroscope();
    imu_.readGyroscope(&gyro_.x(), &gyro_.y(), &gyro_.z());

    // 角速度が大きすぎる場合はやり直し
    if (gyro_.norm() > kStaticGyroThreshold)
    {
      TOBAS_WARN("Perturbation is detected while measuring gyro bias: ", gyro_.transpose(), " [rad/s]. Retrying...");
      i = -1;  // これで次のループで0からやり直しになる
      for (size_t i = 0; i < 3; ++i)
        gyro_sum[i].reset();
      rate.sleep();
      continue;
    }

    // 角速度を加算
    for (size_t i = 0; i < 3; ++i)
      gyro_sum[i].add(gyro_(i));

    rate.sleep();
  }

  // 角速度の平均を計算
  for (size_t i = 0; i < 3; ++i)
    gyro_bias_(i) = gyro_sum[i].get() / kMeasureGyroBiasCount;

  TOBAS_INFO("Finished measuring gyro bias. It is estimated to be: ", gyro_bias_.transpose());
}

void ImuHandler::initializeNoiseFilters()
{
  imu_.updateAccelerometer();
  imu_.updateGyroscope();
  imu_.readAccelerometer(&acc_.x(), &acc_.y(), &acc_.z());
  imu_.readGyroscope(&gyro_.x(), &gyro_.y(), &gyro_.z());

  for (size_t i = 0; i < 3; ++i)
  {
    acc_noise_[i].initialize(kWindowSize, kHpfCutoff, acc_(i));
    gyro_noise_[i].initialize(kWindowSize, kHpfCutoff, gyro_(i));
  }
}
}  // namespace tobas_navio_ros
