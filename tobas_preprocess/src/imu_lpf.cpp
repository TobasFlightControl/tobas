#include <tobas_constants/constants.hpp>

#include "../include/tobas_preprocess/imu_lpf.hpp"

using namespace std;

namespace tobas_preprocess
{
ImuLpf::ImuLpf(const rclcpp::NodeOptions& options) : super(name, options)
{
  imu_lpf_pub_ = createPublisher<tobas_msgs::Imu>(tobas::kImuLpfTopic);
  imu_raw_sub_ = createSubscriber(tobas::kImuTopic, &self::imuRawCb, this);
}

void ImuLpf::imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& imu_raw)
{
  if (last_msg_ == nullptr)
  {
    TOBAS_INFO("First raw IMU message is received.");
    gyro_lpf_.initialize(kGyroLpfCutoff, imu_raw->gyro);
    accel_lpf_.initialize(kAccelLpfCutoff, imu_raw->accel);
    last_msg_ = imu_raw;
    return;
  }

  const auto dt = (imu_raw->header.stamp - last_msg_->header.stamp).seconds();
  last_msg_ = imu_raw;

  gyro_lpf_.update(imu_raw->gyro, dt);
  accel_lpf_.update(imu_raw->accel, dt);

  const auto imu_filtered =std::make_unique<tobas_msgs::Imu>(*imu_raw);
  imu_filtered->gyro = gyro_lpf_.getOutput();
  imu_filtered->accel = accel_lpf_.getOutput();
  imu_lpf_pub_->publish(imu_filtered);
}
}  // namespace tobas_preprocess
