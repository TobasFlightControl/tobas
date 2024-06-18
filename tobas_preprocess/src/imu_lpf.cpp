#include <tobas_ros_tools/eigen_conversion.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_preprocess/imu_lpf.hpp"

using namespace std;

namespace tobas_preprocess
{
ImuLpf::ImuLpf(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  imu_lpf_pub_ = nh_.advertise<sensor_msgs::Imu>(tobas::kImuLpfTopic, 1);
  imu_raw_sub_ = nh_.subscribe(tobas::kImuTopic, 1, &self::imuRawCb, this, tcpNoDelay());
}

void ImuLpf::imuRawCb(const sensor_msgs::ImuConstPtr& imu_raw)
{
  tobas_ros::vectorMsgToEigen(imu_raw->angular_velocity, gyro_);
  tobas_ros::vectorMsgToEigen(imu_raw->linear_acceleration, accel_);

  if (!gyro_lpf_.isInitialized() || !accel_lpf_.isInitialized())
  {
    TOBAS_INFO("First raw IMU message is received.");
    gyro_lpf_.initializeFromCutoff(kGyroLpfCutoff, gyro_);
    accel_lpf_.initializeFromCutoff(kAccelLpfCutoff, accel_);
    t_last_ = imu_raw->header.stamp;
    return;
  }

  const auto ts = (imu_raw->header.stamp - t_last_).toSec();
  t_last_ = imu_raw->header.stamp;

  gyro_lpf_.update(gyro_, ts);
  accel_lpf_.update(accel_, ts);

  const auto imu_filtered = boost::make_shared<sensor_msgs::Imu>(*imu_raw);
  tobas_ros::vectorEigenToMsg(gyro_lpf_.getState(), imu_filtered->angular_velocity);
  tobas_ros::vectorEigenToMsg(accel_lpf_.getState(), imu_filtered->linear_acceleration);
  imu_lpf_pub_.publish(imu_filtered);
}
}  // namespace tobas_preprocess
