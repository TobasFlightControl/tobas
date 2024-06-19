#include <tobas_tools/constants.hpp>

#include "../include/tobas_preprocess/imu_lpf.hpp"

using namespace std;

namespace tobas_preprocess
{
ImuLpf::ImuLpf(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  imu_lpf_pub_ = nh_.advertise<tobas_msgs::Imu>(tobas::kImuLpfTopic, 1);
  imu_raw_sub_ = nh_.subscribe(tobas::kImuTopic, 1, &self::imuRawCb, this, tcpNoDelay());
}

void ImuLpf::imuRawCb(const tobas_msgs::ImuConstPtr& imu_raw)
{
  if (!gyro_lpf_.isInitialized() || !accel_lpf_.isInitialized())
  {
    TOBAS_INFO("First raw IMU message is received.");
    gyro_lpf_.initializeFromCutoff(kGyroLpfCutoff, imu_raw->gyro);
    accel_lpf_.initializeFromCutoff(kAccelLpfCutoff, imu_raw->accel);
    t_last_ = imu_raw->header.stamp;
    return;
  }

  const auto ts = (imu_raw->header.stamp - t_last_).toSec();
  t_last_ = imu_raw->header.stamp;

  gyro_lpf_.update(imu_raw->gyro, ts);
  accel_lpf_.update(imu_raw->accel, ts);

  const auto imu_filtered = boost::make_shared<tobas_msgs::Imu>(*imu_raw);
  imu_filtered->gyro = gyro_lpf_.getState();
  imu_filtered->accel = accel_lpf_.getState();
  imu_lpf_pub_.publish(imu_filtered);
}
}  // namespace tobas_preprocess
