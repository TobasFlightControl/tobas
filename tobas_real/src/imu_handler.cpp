#include <boost/property_tree/ini_parser.hpp>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/boost.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/rate.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_real/imu_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_real
{
ImuHandler::ImuHandler() : super()
{
  getRosParams();
  readConfig();

  setCovarianceMatrices();
  setupImu();
  setGyroBias();
  mag_trans_.initialize();

  imu_msg_.header.frame_id = "imu_frame";
  mag_msg_.header.frame_id = "mag_frame";

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
    const Vector3f acc = acc_ - acc_bias_;  // バイアスを除く
    imu_msg_.linear_acceleration.x = acc.y();
    imu_msg_.linear_acceleration.y = -acc.x();
    imu_msg_.linear_acceleration.z = acc.z();

    imu_.read_gyroscope(&gyro_.x(), &gyro_.y(), &gyro_.z());
    const Vector3f gyro = gyro_ - gyro_bias_;  // バイアスを除く
    imu_msg_.angular_velocity.x = gyro.y();
    imu_msg_.angular_velocity.y = -gyro.x();
    imu_msg_.angular_velocity.z = gyro.z();

    imu_.read_magnetometer(&mag_.x(), &mag_.y(), &mag_.z());
    const Vector3d mag = mag_trans_.transform(mag_.cast<double>());  // 原点中心の単位球に射影
    mag_msg_.magnetic_field.x = mag.x();
    mag_msg_.magnetic_field.y = -mag.y();
    mag_msg_.magnetic_field.z = -mag.z();

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

void ImuHandler::readConfig()
{
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(kConfigPath, pt);

  acc_bias_.x() = pt.get<float>(kConfigKey_AccOffsetX);
  acc_bias_.y() = pt.get<float>(kConfigKey_AccOffsetY);
  acc_bias_.z() = pt.get<float>(kConfigKey_AccOffsetZ);

  mag_trans_.a_xx = pt.get<float>(kConfigKey_MagEllipseAxx);
  mag_trans_.a_yy = pt.get<float>(kConfigKey_MagEllipseAyy);
  mag_trans_.a_zz = pt.get<float>(kConfigKey_MagEllipseAzz);
  mag_trans_.a_xy = pt.get<float>(kConfigKey_MagEllipseAxy);
  mag_trans_.a_yz = pt.get<float>(kConfigKey_MagEllipseAyz);
  mag_trans_.a_zx = pt.get<float>(kConfigKey_MagEllipseAzx);
  mag_trans_.b_x = pt.get<float>(kConfigKey_MagEllipseBx);
  mag_trans_.b_y = pt.get<float>(kConfigKey_MagEllipseBy);
  mag_trans_.b_z = pt.get<float>(kConfigKey_MagEllipseBz);
  mag_trans_.c = pt.get<float>(kConfigKey_MagEllipseC);
}

void ImuHandler::setCovarianceMatrices()
{
  // Accelerometer
  const double acc_std_grav = kAccNoiseDensity * sqrt(kUpdateRate);  // ug
  const double acc_std = acc_std_grav * 1e-6 * tobas::kGravity;      // m/s^2
  const double acc_var = dh_std::sqr(acc_std);                       // m^2/s^4
  dh_std::fillMatrix3Diag(imu_msg_.linear_acceleration_covariance, acc_var);

  // Gyroscope
  const double gyro_std_deg = kGyroNoiseDensity * sqrt(kUpdateRate);  // deg/s
  const double gyro_std_rad = dh_std::deg2rad(gyro_std_deg);          // rad/s
  const double gyro_var = dh_std::sqr(gyro_std_rad);                  // rad^2/s^2
  dh_std::fillMatrix3Diag(imu_msg_.angular_velocity_covariance, gyro_var);

  const double mag_var = dh_std::sqr(kMagNoiseStd);
  dh_std::fillMatrix3Diag(mag_msg_.magnetic_field_covariance, mag_var);
}

void ImuHandler::setupImu()
{
  imu_.initialize();
  if (!imu_.probe())
  {
    rosthrow("IMU not enabled.");
  }
}

void ImuHandler::setGyroBias()
{
  rosInfo("Measuring gyro bias. Please do not move the aircraft.");

  Vector3f gyro_sum = Vector3f::Zero();
  uint32_t cnt = 0;

  dh_ros::Rate rate(kMeasureGyroBiasRate);
  while (cnt++ < kMeasureGyroBiasCount)
  {
    imu_.update();
    imu_.read_gyroscope(&gyro_.x(), &gyro_.y(), &gyro_.z());

    if (gyro_.norm() > kStaticGyroThreshold)
    {
      rosError("Movement of the aircraft is detected while measuring gyro bias.");
      gyro_sum.setZero();
      cnt = 0;
      continue;
    }

    gyro_sum += gyro_;
    rate.sleep();
  }

  gyro_bias_ = gyro_sum / kMeasureGyroBiasCount;

  rosInfo("Finished measuring gyro bias. It is estimated to be:\n" << gyro_bias_);
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
