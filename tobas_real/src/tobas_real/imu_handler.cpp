#include <boost/property_tree/ini_parser.hpp>

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/rate.hpp>

#include <tobas_tools/constants.hpp>

#include "../../include/tobas_real/imu_handler.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_real
{
ImuHandler::ImuHandler() : super()
{
  getRosParams();

  setupImu();
  setCovarianceMatrices();
  getAccelOffset();

  registerPublishers();
  registerSubscribers();
}

void ImuHandler::run()
{
  dh_ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
    imu_msg_.header.stamp = now;
    mag_msg_.header.stamp = now;

    // 各センサのメッセージを更新
    // センサの座標系をNWU座標系に変換する
    imu_.update();

    imu_.read_accelerometer(&acc_.x(), &acc_.y(), &acc_.z());
    const Vector3f acc = acc_ - acc_offset_;  // 事前に計測したオフセットを引く
    imu_msg_.linear_acceleration.x = acc.y();
    imu_msg_.linear_acceleration.y = -acc.x();
    imu_msg_.linear_acceleration.z = acc.z();

    imu_.read_gyroscope(&gyro_.x(), &gyro_.y(), &gyro_.z());
    imu_msg_.angular_velocity.x = gyro_.y();
    imu_msg_.angular_velocity.y = -gyro_.x();
    imu_msg_.angular_velocity.z = gyro_.z();

    imu_.read_magnetometer(&mag_.x(), &mag_.y(), &mag_.z());
    mag_msg_.magnetic_field.x = mag_.x();
    mag_msg_.magnetic_field.y = -mag_.y();
    mag_msg_.magnetic_field.z = -mag_.z();

    // Publish messages
    imu_pub_.publish(imu_msg_);
    mag_pub_.publish(mag_msg_);

    ros::spinOnce();
    rate.sleep();
  }
}

void ImuHandler::getRosParams()
{
}

void ImuHandler::registerPublishers()
{
  imu_pub_ = nh_.advertise<ImuMsg>("imu", 1);
  mag_pub_ = nh_.advertise<MagMsg>("magnetic_field", 1);
}

void ImuHandler::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &ImuHandler::eventCb, this);
}

void ImuHandler::setupImu()
{
  if (!imu_.probe())
  {
    rosthrow("Sensor not enabled.");
  }

  imu_.initialize();
}

void ImuHandler::setCovarianceMatrices()
{
  // Accelerometer
  double acc_std_grav = kAccNoiseDensity * sqrt(kUpdateRate);  // ug
  double acc_std = acc_std_grav * 1e-6 * tobas::kGravity;      // m/s^2
  double acc_var = dh_std::sqr(acc_std);                       // m^2/s^4
  imu_msg_.linear_acceleration_covariance[0] = acc_var;
  imu_msg_.linear_acceleration_covariance[4] = acc_var;
  imu_msg_.linear_acceleration_covariance[8] = acc_var;

  // Gyroscope
  double gyro_std_deg = kGyroNoiseDensity * sqrt(kUpdateRate);  // deg/s
  double gyro_std_rad = dh_std::deg2rad(gyro_std_deg);          // rad/s
  double gyro_var = dh_std::sqr(gyro_std_rad);                  // rad^2/s^2
  imu_msg_.angular_velocity_covariance[0] = gyro_var;
  imu_msg_.angular_velocity_covariance[4] = gyro_var;
  imu_msg_.angular_velocity_covariance[8] = gyro_var;

  double mag_var = dh_std::sqr(kMagNoiseStd);
  mag_msg_.magnetic_field_covariance[0] = mag_var;
  mag_msg_.magnetic_field_covariance[4] = mag_var;
  mag_msg_.magnetic_field_covariance[8] = mag_var;
}

void ImuHandler::getAccelOffset()
{
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(kConfigPath, pt);

  acc_offset_.x() = pt.get<float>(kConfigKey_AccOffsetX);
  acc_offset_.y() = pt.get<float>(kConfigKey_AccOffsetY);
  acc_offset_.z() = pt.get<float>(kConfigKey_AccOffsetZ);
}

void ImuHandler::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      // ros::shutdown();
      break;
    default:
      break;
  }
}
}  // namespace tobas_real
