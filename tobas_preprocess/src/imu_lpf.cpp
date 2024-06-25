#include <tobas_tools/constants.hpp>

#include "../include/tobas_preprocess/imu_lpf.hpp"

using namespace std;

namespace tobas_preprocess
{
ImuLpf::ImuLpf(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  imu_lpf_pub_ = nh_.advertise<tobas_msgs::Imu>(tobas::kImuLpfTopic, 1);
  imu_raw_sub_ = nh_.subscribe(tobas::kImuTopic, 1, &self::imuRawCb, this, tcpNoDelay());
}

void ImuLpf::imuRawCb(const tobas_msgs::ImuConstPtr& imu_raw)
{
  if (last_msg_ == nullptr)
  {
    TOBAS_INFO("First raw IMU message is received.");
    gyro_lpf_.initializeFromCutoff(kGyroLpfCutoff, imu_raw->gyro);
    accel_lpf_.initializeFromCutoff(kAccelLpfCutoff, imu_raw->accel);
    last_msg_ = imu_raw;
    return;
  }

  const auto dt = (imu_raw->header.stamp - last_msg_->header.stamp).toSec();
  last_msg_ = imu_raw;

  gyro_lpf_.update(imu_raw->gyro, dt);
  accel_lpf_.update(imu_raw->accel, dt);

  const auto imu_filtered = boost::make_shared<tobas_msgs::Imu>(*imu_raw);
  imu_filtered->gyro = gyro_lpf_.getOutput();
  imu_filtered->accel = accel_lpf_.getOutput();
  imu_lpf_pub_.publish(imu_filtered);
}
}  // namespace tobas_preprocess
