#include <boost/property_tree/ini_parser.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/boost.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../include/tobas_real/imu_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_real
{
ImuHandler::ImuHandler(ros::NodeHandle nh, ros::NodeHandle pnh, string name) : super(nh, pnh, name)
{
  getRosParams();
  readConfig();

  setupImu();
  measureGyroBias();
  mag_trans_.initialize();

  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(ros::Duration(1 / kUpdateRate), &ImuHandler::mainTimerCb, this);
}

void ImuHandler::getRosParams()
{
}

void ImuHandler::registerPublishers()
{
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
  mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);
}

void ImuHandler::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &ImuHandler::eventCb, this, tcpNoDelay());
}

void ImuHandler::readConfig()
{
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(kConfigPath, pt);

  acc_noise_density_ = pt.get<double>(kConfigKey_AccNoiseDensity);
  gyro_noise_density_ = pt.get<double>(kConfigKey_GyroNoiseDensity);
  mag_noise_density_ = pt.get<double>(kConfigKey_MagNoiseDensity);

  acc_bias_.x() = pt.get<float>(kConfigKey_AccOffsetX);
  acc_bias_.y() = pt.get<float>(kConfigKey_AccOffsetY);
  acc_bias_.z() = pt.get<float>(kConfigKey_AccOffsetZ);

  mag_trans_.a_xx = pt.get<double>(kConfigKey_MagEllipseAxx);
  mag_trans_.a_yy = pt.get<double>(kConfigKey_MagEllipseAyy);
  mag_trans_.a_zz = pt.get<double>(kConfigKey_MagEllipseAzz);
  mag_trans_.a_xy = pt.get<double>(kConfigKey_MagEllipseAxy);
  mag_trans_.a_yz = pt.get<double>(kConfigKey_MagEllipseAyz);
  mag_trans_.a_zx = pt.get<double>(kConfigKey_MagEllipseAzx);
  mag_trans_.b_x = pt.get<double>(kConfigKey_MagEllipseBx);
  mag_trans_.b_y = pt.get<double>(kConfigKey_MagEllipseBy);
  mag_trans_.b_z = pt.get<double>(kConfigKey_MagEllipseBz);
  mag_trans_.c = pt.get<double>(kConfigKey_MagEllipseC);
}

void ImuHandler::setupImu()
{
  imu_.initialize();
  if (!imu_.probe())
  {
    rosthrow(name_, "IMU not enabled.");
  }
}

void ImuHandler::measureGyroBias()
{
  rosInfo(name_, "Measuring gyro bias. Please do not move the aircraft.");

  Vector3f gyro_sum = Vector3f::Zero();
  uint32_t cnt = 0;

  dh_ros::Rate rate(kMeasureGyroBiasRate);
  while (cnt++ < kMeasureGyroBiasCount)
  {
    imu_.update();
    imu_.read_gyroscope(&gyro_.x(), &gyro_.y(), &gyro_.z());

    if (gyro_.norm() > kStaticGyroThreshold)
    {
      rosError(
        name_,
        "Movement of the aircraft is detected while measuring gyro bias. The norm of gyro is "
          << gyro_.norm() << " rad/s.");
      gyro_sum.setZero();
      cnt = 0;
      rate.sleep();
      continue;
    }

    gyro_sum += gyro_;
    rate.sleep();
  }

  gyro_bias_ = gyro_sum / kMeasureGyroBiasCount;

  rosInfo(name_, "Finished measuring gyro bias. It is estimated to be:\n" << gyro_bias_);
}

void ImuHandler::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      // nh_.shutdown();
      break;
    default:
      break;
  }
}

void ImuHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // rosInfo(name_, "IMU timer callback is called.");

  // Update IMU
  imu_.update();

  // Read IMU
  imu_.read_accelerometer(&acc_.x(), &acc_.y(), &acc_.z());
  imu_.read_gyroscope(&gyro_.x(), &gyro_.y(), &gyro_.z());
  imu_.read_magnetometer(&mag_.x(), &mag_.y(), &mag_.z());

  // Create messages
  const auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
  const auto mag_msg = boost::make_shared<sensor_msgs::MagneticField>();

  // Fill headers
  imu_msg->header.stamp = event.current_real;
  mag_msg->header.stamp = event.current_real;
  imu_msg->header.frame_id = "imu_frame";
  mag_msg->header.frame_id = "mag_frame";

  // Fill covariance matrices
  const auto acc_var = dh_std::sqr(acc_noise_density_) * kUpdateRate;    // [m^2/s^4]
  const auto gyro_var = dh_std::sqr(gyro_noise_density_) * kUpdateRate;  // [rad^2/s^2]
  const auto mag_var = dh_std::sqr(mag_noise_density_) * kUpdateRate;
  dh_std::fillMatrix3Diag(imu_msg->linear_acceleration_covariance, acc_var);
  dh_std::fillMatrix3Diag(imu_msg->angular_velocity_covariance, gyro_var);
  dh_std::fillMatrix3Diag(mag_msg->magnetic_field_covariance, mag_var);

  // Fill data (Convert to NWU coordinate system)
  const Vector3f acc = acc_ - acc_bias_;  // バイアスを除く
  imu_msg->linear_acceleration.x = acc.y();
  imu_msg->linear_acceleration.y = -acc.x();
  imu_msg->linear_acceleration.z = acc.z();

  const Vector3f gyro = gyro_ - gyro_bias_;  // バイアスを除く
  imu_msg->angular_velocity.x = gyro.y();
  imu_msg->angular_velocity.y = -gyro.x();
  imu_msg->angular_velocity.z = gyro.z();

  const Vector3d mag = mag_trans_.transform(mag_.cast<double>());  // 原点中心の単位球に射影
  mag_msg->magnetic_field.x = mag.x();
  mag_msg->magnetic_field.y = -mag.y();
  mag_msg->magnetic_field.z = -mag.z();

  // Publish messages
  imu_pub_.publish(imu_msg);
  mag_pub_.publish(mag_msg);
}
}  // namespace tobas_real
