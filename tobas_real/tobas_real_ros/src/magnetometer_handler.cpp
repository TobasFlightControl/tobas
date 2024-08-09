#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_msgs/MagneticField.hpp>

#include "../include/tobas_real_ros/magnetometer_handler.hpp"
#include "../include/tobas_real_ros/common.hpp"

using namespace std;

namespace tobas_real_ros
{
MagnetometerHandler::MagnetometerHandler(const rclcpp::NodeOptions& options)
  : super(node, pnh, name), property_client_(node_, kPropertyServerFC)
{
  reloadConfig();

  mag_pub_ = createPublisher<tobas_msgs::MagneticField>(tobas::kMagTopic);
  mag_sub_ = createSubscriber(hal::kMagTopic, &self::magCb, this);

  reload_config_srv_ = createService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);
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

void MagnetometerHandler::magCb(const tobas_hal_msgs::MagneticField::ConstSharedPtr& mag_raw)
{
  // Project data to unit sphere
  const auto mag_unit = mag_trans_.transform(mag_raw->magnetic_field.data);

  // Initialize
  if (mag_raw_ == nullptr)
  {
    for (size_t i = 0; i < 3; ++i)
      mag_noise_[i].initialize(kWindowSize, kHpfCutoff, mag_unit(i));
    mag_raw_ = mag_raw;
    return;
  }

  // Compute time difference
  const auto dt = (mag_raw->header.stamp - mag_raw_->header.stamp).seconds();
  mag_raw_ = mag_raw;

  // Update noise filter
  for (size_t i = 0; i < 3; ++i)
    mag_noise_[i].update(mag_unit(i), dt);

  // Create message
  const auto mag_msg =std::make_unique<tobas_msgs::MagneticField>();

  // Fill header
  mag_msg->header = mag_raw->header;

  // Fill data
  mag_msg->magnetic_field.data = mag_unit;

  // Fill covariance matrices
  mag_msg->covariance.setZero();
  for (size_t i = 0; i < 3; ++i)
    mag_msg->covariance(i, i) = mag_noise_[i].noiseVariance();

  // Publish message
  mag_pub_->publish(mag_msg);
}

bool MagnetometerHandler::reloadConfigCb(std_srvs::srv::Trigger::Request&, std_srvs::srv::Trigger::Response& res)
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
}  // namespace tobas_real_ros
