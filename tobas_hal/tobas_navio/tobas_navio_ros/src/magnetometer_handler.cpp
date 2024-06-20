#include <tobas_math/core.hpp>
#include <tobas_std_tools/property_tree.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/MagneticField.h>

#include "../include/tobas_navio_ros/magnetometer_handler.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_navio_ros
{
MagnetometerHandler::MagnetometerHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  PRINT_DEBUG("MagnetometerHandler::MagnetometerHandler");

  reloadConfig();

  imu_.initialize();
  if (!imu_.probe())
    TOBAS_EXIT("IMU not enabled.");

  mag_pub_ = nh_.advertise<tobas_msgs::MagneticField>(tobas::kMagTopic, 1);
  reload_config_srv_ = nh_.advertiseService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);

  PRINT_DEBUG("/MagnetometerHandler::MagnetometerHandler");
}

bool MagnetometerHandler::reloadConfig()
{
  // 設定が取得できなかった場合でも最低限初期化しないとまずいため，途中でリターンせず返り値を保持しておく．
  tobas_std::PropertyTree pt(kConfigPath);

  bool res = true;

  if (!pt.get(kConfigKey_MagNoiseDensity, mag_noise_density_, kDefaultMagNoiseDensity))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_MagNoiseDensity, ".");
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

  mag_var_ = math::sqr(mag_noise_density_) * kSamplingRate;  // TODO: スケーリング

  if (!mag_trans_.initialize())
  {
    TOBAS_ERROR("Failed to initialize ellipse transformer.");
    res = false;
  }

  return res;
}

bool MagnetometerHandler::reloadConfigCb(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
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

void MagnetometerHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Update IMU
  imu_.updateMagnetometer();

  // Read IMU
  imu_.readMagnetometer(&mag_.x(), &mag_.y(), &mag_.z());

  // Create messages
  const auto mag_msg = boost::make_shared<tobas_msgs::MagneticField>();

  // Fill headers
  mag_msg->header.stamp = event.current_real;

  // Fill covariance matrices
  mag_msg->covariance = Vector3d::Constant(mag_var_).asDiagonal();

  // Fill data (Convert to NWU coordinate system)
  const auto mag = mag_trans_.transform(mag_.cast<double>());  // 単位球に射影
  mag_msg->magnetic_field.x(mag.x());
  mag_msg->magnetic_field.y(-mag.y());
  mag_msg->magnetic_field.z(-mag.z());

  // Publish messages
  mag_pub_.publish(mag_msg);
}
}  // namespace tobas_navio_ros
