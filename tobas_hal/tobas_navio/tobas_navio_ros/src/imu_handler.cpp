#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_std_tools/boost.hpp>
#include <tobas_std_tools/property_tree.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_navio_ros/imu_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;
using namespace Eigen;
using namespace tobas_std;

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
  mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>(tobas::kMagTopic, 1);

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
  PropertyTree pt(kConfigPath);

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
  if (!pt.get(kConfigKey_MagNoiseDensity, mag_noise_density_, kDefaultMagNoiseDensity))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagNoiseDensity, ".");
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
  if (!pt.get(kConfigKey_MagEllipseAxx, mag_trans_.a_xx, 1.))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagEllipseAxx, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_MagEllipseAyy, mag_trans_.a_yy, 1.))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagEllipseAyy, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_MagEllipseAzz, mag_trans_.a_zz, 1.))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagEllipseAzz, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_MagEllipseAxy, mag_trans_.a_xy, 0.))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagEllipseAxy, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_MagEllipseAyz, mag_trans_.a_yz, 0.))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagEllipseAyz, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_MagEllipseAzx, mag_trans_.a_zx, 0.))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagEllipseAzx, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_MagEllipseBx, mag_trans_.b_x, 0.))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagEllipseBx, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_MagEllipseBy, mag_trans_.b_y, 0.))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagEllipseBy, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_MagEllipseBz, mag_trans_.b_z, 0.))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagEllipseBz, ".");
    res = false;
  }
  if (!pt.get(kConfigKey_MagEllipseC, mag_trans_.c, -1.))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagEllipseC, ".");
    res = false;
  }

  acc_var_ = sqr(acc_noise_density_) * kSamplingRate;    // [m^2/s^4]
  gyro_var_ = sqr(gyro_noise_density_) * kSamplingRate;  // [rad^2/s^2]
  mag_var_ = sqr(mag_noise_density_) * kSamplingRate;    // TODO: スケーリング

  if (!mag_trans_.initialize())
  {
    TOBAS_ERROR("Failed to initialize ellipse transformer.");
    res = false;
  }

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
  imu_.updateMagnetometer();

  // Read IMU
  imu_.readAccelerometer(&acc_.x(), &acc_.y(), &acc_.z());
  imu_.readGyroscope(&gyro_.x(), &gyro_.y(), &gyro_.z());
  imu_.readMagnetometer(&mag_.x(), &mag_.y(), &mag_.z());

  // Create messages
  const auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
  const auto mag_msg = boost::make_shared<sensor_msgs::MagneticField>();

  // Fill headers
  imu_msg->header.stamp = event.current_real;
  mag_msg->header.stamp = event.current_real;

  // Fill covariance matrices
  fillMatrix3Diag(imu_msg->linear_acceleration_covariance, acc_var_);
  fillMatrix3Diag(imu_msg->angular_velocity_covariance, gyro_var_);
  fillMatrix3Diag(mag_msg->magnetic_field_covariance, mag_var_);

  // Fill data (Convert to NWU coordinate system)
  const Vector3f acc = acc_ - acc_bias_;  // バイアスを除く
  imu_msg->linear_acceleration.x = acc.y();
  imu_msg->linear_acceleration.y = -acc.x();
  imu_msg->linear_acceleration.z = acc.z();

  const Vector3f gyro = gyro_ - gyro_bias_;  // バイアスを除く
  imu_msg->angular_velocity.x = gyro.y();
  imu_msg->angular_velocity.y = -gyro.x();
  imu_msg->angular_velocity.z = gyro.z();

  const Vector3d mag = mag_trans_.transform(mag_.cast<double>());  // 単位球に射影
  mag_msg->magnetic_field.x = mag.x();
  mag_msg->magnetic_field.y = -mag.y();
  mag_msg->magnetic_field.z = -mag.z();

  // Publish messages
  imu_pub_.publish(imu_msg);
  mag_pub_.publish(mag_msg);
}

void ImuHandler::measureGyroBiasTimerCb(const ros::TimerEvent&)
{
  if (cnt_ == kMeasureGyroBiasCount)
  {
    for (size_t i = 0; i < 3; ++i)
      gyro_bias_(i) = fmean(gyro_buf_[i]);
    TOBAS_INFO("Finished measuring gyro bias. It is estimated to be: ", gyro_bias_.transpose());
    measure_gyro_bias_timer_.stop();
    main_timer_.start();
    return;
  }

  imu_.updateGyroscope();
  imu_.readGyroscope(&gyro_buf_[0][cnt_], &gyro_buf_[1][cnt_], &gyro_buf_[2][cnt_]);

  if (sqrt(sqr(gyro_buf_[0][cnt_]) + sqr(gyro_buf_[1][cnt_]) + sqr(gyro_buf_[2][cnt_])) > kStaticGyroThreshold)
  {
    TOBAS_WARN("Perturbation is detected while measuring gyro bias: ", gyro_.transpose(), " [rad/s]. Retrying...");
    cnt_ = 0;
    return;
  }

  ++cnt_;
}
}  // namespace tobas_navio_ros
