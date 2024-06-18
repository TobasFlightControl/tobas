#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/boost.hpp>
#include <tobas_std_tools/property_tree.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_navio_ros/imu_handler.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_navio_ros
{
ImuHandler::ImuHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  PRINT_DEBUG("ImuHandler::ImuHandler");

  reloadConfig();

  imu_.initialize();
  if (!imu_.probe())
    TOBAS_EXIT("IMU not enabled.");

  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(tobas::kImuTopic, 1);
  reload_config_srv_ = nh_.advertiseService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);

  // まずジャイロのバイアスを測定する
  // コンストラクタで時間をとると他のNodeletがスタックするため，タイマーコールバックで行う
  measure_gyro_bias_timer_ = nh_.createTimer(kSamplingRate, &self::measureGyroBiasTimerCb, this);

  // メインタイマーはジャイロのバイアスが測定してからスタートする
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this, false, false);

  PRINT_DEBUG("/ImuHandler::ImuHandler");
}

bool ImuHandler::reloadConfig()
{
  // 設定が取得できなかった場合でも最低限初期化しないとまずいため，途中でリターンせず返り値を保持しておく．
  tobas_std::PropertyTree pt(kConfigPath);

  bool res = true;

  if (!pt.get(kConfigKey_AccNoiseDensity, acc_noise_density_, kDefaultAccNoiseDensity))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_AccNoiseDensity, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_GyroNoiseDensity, gyro_noise_density_, kDefaultGyroNoiseDensity))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_GyroNoiseDensity, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_AccOffsetX, acc_bias_.x(), 0.0f))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_AccOffsetX, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_AccOffsetY, acc_bias_.y(), 0.0f))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_AccOffsetY, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_AccOffsetZ, acc_bias_.z(), 0.0f))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_AccOffsetZ, ".");
    res = false;
  }

  acc_var_ = tobas_std::sqr(acc_noise_density_) * kSamplingRate;    // [m^2/s^4]
  gyro_var_ = tobas_std::sqr(gyro_noise_density_) * kSamplingRate;  // [rad^2/s^2]

  return res;
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

  // Create messages
  const auto imu_msg = boost::make_shared<sensor_msgs::Imu>();

  // Fill headers
  imu_msg->header.stamp = event.current_real;

  // Fill covariance matrices
  tobas_std::fillMatrix3Diag(imu_msg->linear_acceleration_covariance, acc_var_);
  tobas_std::fillMatrix3Diag(imu_msg->angular_velocity_covariance, gyro_var_);

  // Fill data (Convert to NWU coordinate system)
  const Vector3f acc = acc_ - acc_bias_;  // バイアスを除く
  imu_msg->linear_acceleration.x = acc.y();
  imu_msg->linear_acceleration.y = -acc.x();
  imu_msg->linear_acceleration.z = acc.z();

  const Vector3f gyro = gyro_ - gyro_bias_;  // バイアスを除く
  imu_msg->angular_velocity.x = gyro.y();
  imu_msg->angular_velocity.y = -gyro.x();
  imu_msg->angular_velocity.z = gyro.z();

  // Publish messages
  imu_pub_.publish(imu_msg);
}

void ImuHandler::measureGyroBiasTimerCb(const ros::TimerEvent&)
{
  if (loop_cnt_ == kMeasureGyroBiasCount)
  {
    gyro_bias_ = gyro_sum_ / kMeasureGyroBiasCount;
    TOBAS_INFO("Finished measuring gyro bias. It is estimated to be: ", gyro_bias_.transpose());
    measure_gyro_bias_timer_.stop();
    main_timer_.start();
    return;
  }

  imu_.updateGyroscope();
  imu_.readGyroscope(&gyro_.x(), &gyro_.y(), &gyro_.z());

  if (gyro_.norm() > kStaticGyroThreshold)
  {
    TOBAS_WARN("Perturbation is detected while measuring gyro bias: ", gyro_.transpose(), " [rad/s]. Retrying...");
    loop_cnt_ = 0;
    gyro_sum_.setZero();
    gyro_c_.setZero();
    return;
  }

  // Kahan summation
  const Vector3f y = gyro_ - gyro_c_;
  const Vector3f t = gyro_sum_ + y;
  gyro_c_ = (t - gyro_sum_) - y;
  gyro_sum_ = t;

  ++loop_cnt_;
}
}  // namespace tobas_navio_ros
