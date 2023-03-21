#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>

#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>

#include "../../include/multirotor_real/imu_handler.hpp"

#define GRAVITY 9.80665
#define TIMER_PERIOD 1e-2

// MPU9250
// https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
#define ACC_NOISE_DENSITY 300.   // ug/sqrt(hz)
#define GYRO_NOISE_DENSITY 0.01  // deg/s/sqrt(hz)
#define MAG_NOISE_STD 0.  // TODO: データシートに無かったため，磁気センサの精度を計測する

using namespace std;

ImuHandler::ImuHandler(ros::NodeHandle& nh)
{
  setupImu();
  setCovarianceMatrices();
  advertisePublishers(nh);

  timer_ = nh.createTimer(ros::Duration(TIMER_PERIOD), &ImuHandler::timerCb, this);
}

void ImuHandler::setupImu()
{
  imu_ = ImuPtr(new MPU9250());
  // imu_ = ImuPtr(new LSM9DS1());

  if (!imu_->probe())
  {
    throw dh_ros::RuntimeError("Sensor not enabled.");
  }

  imu_->initialize();
}

void ImuHandler::setCovarianceMatrices()
{
  // Accelerometer
  double acc_std_grav = ACC_NOISE_DENSITY / sqrt(TIMER_PERIOD);  // ug
  double acc_std = acc_std_grav * 1e-6 * GRAVITY;                // m/s^2
  double acc_var = dh_std::sqr(acc_std);                         // m^2/s^4
  imu_msg_.linear_acceleration_covariance[0] = acc_var;
  imu_msg_.linear_acceleration_covariance[4] = acc_var;
  imu_msg_.linear_acceleration_covariance[8] = acc_var;

  // Gyroscope
  double gyro_std_deg = GYRO_NOISE_DENSITY / sqrt(TIMER_PERIOD);  // deg/s
  double gyro_std_rad = dh_std::deg2rad(gyro_std_deg);            // rad/s
  double gyro_var = dh_std::sqr(gyro_std_rad);                    // rad^2/s^2
  imu_msg_.angular_velocity_covariance[0] = gyro_var;
  imu_msg_.angular_velocity_covariance[4] = gyro_var;
  imu_msg_.angular_velocity_covariance[8] = gyro_var;

  double mag_var = dh_std::sqr(MAG_NOISE_STD);
  mag_msg_.magnetic_field_covariance[0] = mag_var;
  mag_msg_.magnetic_field_covariance[4] = mag_var;
  mag_msg_.magnetic_field_covariance[8] = mag_var;
}

void ImuHandler::advertisePublishers(ros::NodeHandle& nh)
{
  string drone_name = dh_ros::getParam<string>("/drone_name");
  imu_pub_ = nh.advertise<ImuMsg>("/" + drone_name + "/imu", 1);
  mag_pub_ = nh.advertise<MagMsg>("/" + drone_name + "/magnetic_field", 1);
}

void ImuHandler::timerCb(const ros::TimerEvent&)
{
  ros::Time now = ros::Time::now();
  imu_msg_.header.stamp = now;
  mag_msg_.header.stamp = now;

  // 各センサのメッセージを更新
  // センサの座標系をNWU座標系に変換する

  imu_->read_accelerometer(&ax_, &ay_, &az_);
  imu_msg_.linear_acceleration.x = ay_;
  imu_msg_.linear_acceleration.y = -ax_;
  imu_msg_.linear_acceleration.z = az_;

  imu_->read_gyroscope(&wx_, &wy_, &wz_);
  imu_msg_.angular_velocity.x = wy_;
  imu_msg_.angular_velocity.y = -wx_;
  imu_msg_.angular_velocity.z = wz_;

  imu_->read_magnetometer(&mx_, &my_, &mz_);
  mag_msg_.magnetic_field.x = mx_;
  mag_msg_.magnetic_field.y = -my_;
  mag_msg_.magnetic_field.z = -mz_;
}
