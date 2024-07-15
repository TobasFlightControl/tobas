#include <tobas_tools/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_msgs/Imu.h>

#include "../include/tobas_real_ros/imu_handler.hpp"
#include "../include/tobas_real_ros/common.hpp"

using namespace std;

namespace tobas_real_ros
{
ImuHandler::ImuHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), property_client_(nh_, kPropertyServerFC)
{
  reloadConfig();

  imu_pub_ = nh_.advertise<tobas_msgs::Imu>(tobas::kImuTopic, 1);
  imu_sub_ = nh_.subscribe(hal::kImuTopic, 1, &self::imuCb, this, tcpNoDelay());

  reload_config_srv_ = nh_.advertiseService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);
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

void ImuHandler::imuCb(const tobas_hal_msgs::ImuConstPtr& imu_raw)
{
  switch (stage_)
  {
    case MEASURE_GYRO_BIAS:
    {
      // 角速度が大きすぎる場合はやり直し
      if (imu_raw->gyro.norm() > kStaticGyroThreshold)
      {
        TOBAS_WARN("Perturbation is detected while measuring gyro bias: ", imu_raw->gyro, " [rad/s]. Retrying...");
        gyro_bias_cnt_ = 0;
        for (size_t i = 0; i < 3; ++i)
          gyro_sum_[i].reset();
        break;
      }

      // 角速度を加算
      for (size_t i = 0; i < 3; ++i)
        gyro_sum_[i].add(imu_raw->gyro(i));

      // データが溜まったら角速度の平均をバイアスの推定値として次のステージに進む
      if (++gyro_bias_cnt_ == kMeasureGyroBiasCount)
      {
        for (size_t i = 0; i < 3; ++i)
          gyro_bias_(i) = gyro_sum_[i].get() / kMeasureGyroBiasCount;
        TOBAS_INFO("Finished measuring gyro bias. It is estimated to be: ", gyro_bias_);
        stage_ = INITIALIZE;
      }

      break;
    }
    case INITIALIZE:
    {
      for (size_t i = 0; i < 3; ++i)
      {
        acc_noise_[i].initialize(kWindowSize, kHpfCutoff, imu_raw->accel(i));
        gyro_noise_[i].initialize(kWindowSize, kHpfCutoff, imu_raw->gyro(i));
      }
      imu_raw_ = imu_raw;
      stage_ = PUBLISH;
      break;
    }
    case PUBLISH:
    {
      // Compute time difference
      const auto dt = (imu_raw->header.stamp - imu_raw_->header.stamp).toSec();
      imu_raw_ = imu_raw;

      // Update noise filters
      for (size_t i = 0; i < 3; ++i)
      {
        acc_noise_[i].update(imu_raw->accel(i), dt);
        gyro_noise_[i].update(imu_raw->gyro(i), dt);
      }

      // Create message
      const auto imu_msg = boost::make_shared<tobas_msgs::Imu>();

      // Fill header
      imu_msg->header = imu_raw->header;

      // Fill data
      imu_msg->accel = imu_raw->accel - acc_bias_;
      imu_msg->gyro = imu_raw->gyro - gyro_bias_;

      // Fill covariance matrices
      imu_msg->accel_covariance.setZero();
      imu_msg->gyro_covariance.setZero();
      for (size_t i = 0; i < 3; ++i)
      {
        imu_msg->accel_covariance(i, i) = acc_noise_[i].noiseVariance();
        imu_msg->gyro_covariance(i, i) = gyro_noise_[i].noiseVariance();
      }

      // Publish message
      imu_pub_.publish(imu_msg);

      break;
    }
  }
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
}  // namespace tobas_real_ros
