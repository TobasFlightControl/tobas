#include <tobas_math/core.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/MagneticField.h>

#include "../include/tobas_navio_ros/magnetometer_handler.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_navio_ros
{
MagnetometerHandler::MagnetometerHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), property_client_(nh_, kPropertyServerFC)
{
  PRINT_DEBUG("MagnetometerHandler::MagnetometerHandler");

  if (!imu_.probe())
    TOBAS_EXIT("IMU not enabled.");
  imu_.initialize();

  reloadConfig();
  initializeNoiseFilter();

  mag_pub_ = nh_.advertise<tobas_msgs::MagneticField>(tobas::kMagTopic, 1);
  reload_config_srv_ = nh_.advertiseService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);

  PRINT_DEBUG("/MagnetometerHandler::MagnetometerHandler");
}

bool MagnetometerHandler::reloadConfig()
{
  if (property_client_.get(kConfigKey_MagEllipseAxx, mag_trans_.a_xx) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_.get(kConfigKey_MagEllipseAyy, mag_trans_.a_yy) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_.get(kConfigKey_MagEllipseAzz, mag_trans_.a_zz) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_.get(kConfigKey_MagEllipseAxy, mag_trans_.a_xy) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_.get(kConfigKey_MagEllipseAyz, mag_trans_.a_yz) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_.get(kConfigKey_MagEllipseAzx, mag_trans_.a_zx) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_.get(kConfigKey_MagEllipseBx, mag_trans_.b_x) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_.get(kConfigKey_MagEllipseBy, mag_trans_.b_y) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_.get(kConfigKey_MagEllipseBz, mag_trans_.b_z) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_.get(kConfigKey_MagEllipseC, mag_trans_.c) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    mag_trans_.setIdentity();
    return false;
  }

  if (!mag_trans_.initialize())
  {
    TOBAS_ERROR("Failed to initialize ellipse transformer.");
    mag_trans_.setIdentity();
    return false;
  }

  return true;
}

void MagnetometerHandler::initializeNoiseFilter()
{
  imu_.updateMagnetometer();
  imu_.readMagnetometer(&mag_.x(), &mag_.y(), &mag_.z());

  constexpr size_t window_size = kSamplingRate * kNoiseStatTimeWindow / 1000;
  for (size_t i = 0; i < 3; ++i)
    mag_noise_[i].initialize(window_size, kHpfCutoff, mag_(i));
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

  // Update noise filter
  const auto dt = (event.current_real - event.last_real).toSec();
  for (size_t i = 0; i < 3; ++i)
    mag_noise_[i].update(mag_(i), dt);

  // Create messages
  const auto mag_msg = boost::make_shared<tobas_msgs::MagneticField>();

  // Fill headers
  mag_msg->header.stamp = event.current_real;

  // Fill covariance matrices
  mag_msg->covariance.setZero();
  for (size_t i = 0; i < 3; ++i)
    mag_msg->covariance(i, i) = mag_noise_[i].noiseVariance();

  // Fill data (Convert to NWU coordinate system)
  const auto mag = mag_trans_.transform(mag_.cast<double>());  // 単位球に射影
  mag_msg->magnetic_field.x(mag.x());
  mag_msg->magnetic_field.y(-mag.y());
  mag_msg->magnetic_field.z(-mag.z());

  // Publish messages
  mag_pub_.publish(mag_msg);
}
}  // namespace tobas_navio_ros
