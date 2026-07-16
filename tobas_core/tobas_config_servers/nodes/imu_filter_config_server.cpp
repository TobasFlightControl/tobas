// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/node.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/srv/configure_imu_low_pass_filter.hpp>
#include <tobas_msgs/srv/configure_imu_rpm_filter.hpp>
#include <tobas_msgs_adapter/imu.hpp>

namespace tobas
{
class ImuFilterConfigServer : public BaseNode
{
  using self = ImuFilterConfigServer;
  using super = BaseNode;

public:
  explicit ImuFilterConfigServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Dynamic parameters
  struct LowPassFilterConfig
  {
    int accel_cutoff = -1;
    int gyro_cutoff = -1;
    int dgyro_cutoff = -1;
  } lowpass_cfg_;
  struct NotchFilterConfig
  {
    int quality_factor = -1;
    int min_center_freq = -1;
    int fade_range = -1;
    int lpf_cutoff = -1;
  } notch_cfg_;

  ros2::SubscriberPtr<tobas_msgs::Imu> imu_raw_sub_;
  ros2::ServiceClientPtr<tobas_msgs::srv::ConfigureImuLowPassFilter> config_lowpass_filter_sc_;
  ros2::ServiceClientPtr<tobas_msgs::srv::ConfigureImuRpmFilter> config_rpm_filter_sc_;

  bool lowPassFilterConfigReady() const;
  bool notchFilterConfigReady() const;
  bool sendLowPassFilterConfigRequest();
  bool sendRpmFilterConfigRequest();

  bool lowPassFilterAccelCutoffCb(const long& p);
  bool lowPassFilterGyroCutoffCb(const long& p);
  bool lowPassFilterDGyroCutoffCb(const long& p);
  bool rpmFilterQualityFactorCb(const long& p);
  bool rpmFilterMinCenterFreqCb(const long& p);
  bool rpmFilterFadeRangeCb(const long& p);
  bool rpmFilterLpfCutoffCb(const long& p);

  void imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& msg);
};

ImuFilterConfigServer::ImuFilterConfigServer(const rclcpp::NodeOptions& options)
  : super(node::kImuFilterConfigServer, nodeOptions_DParam(options))
{
  const auto has_rpm_filter = getBoolParam("has_rpm_filter");

  imu_raw_sub_ = createSubscriber(topic::kImuRaw, &self::imuRawCb, this);

  config_lowpass_filter_sc_ =
    create_client<tobas_msgs::srv::ConfigureImuLowPassFilter>(service::kConfigureImuLowPassFilter);

  if (has_rpm_filter) {
    config_rpm_filter_sc_ = create_client<tobas_msgs::srv::ConfigureImuRpmFilter>(service::kConfigureImuRpmFilter);
  }
}

bool ImuFilterConfigServer::lowPassFilterConfigReady() const
{
  return lowpass_cfg_.accel_cutoff >= 0 && lowpass_cfg_.gyro_cutoff >= 0 && lowpass_cfg_.dgyro_cutoff >= 0;
}

bool ImuFilterConfigServer::notchFilterConfigReady() const
{
  return notch_cfg_.quality_factor >= 0 && notch_cfg_.min_center_freq >= 0 && notch_cfg_.fade_range >= 0 &&
         notch_cfg_.lpf_cutoff >= 0;
}

bool ImuFilterConfigServer::sendLowPassFilterConfigRequest()
{
  if (!config_lowpass_filter_sc_->service_is_ready()) {
    TOBAS_ERROR("\"", service::kConfigureImuLowPassFilter, "\" is not ready.");
    return false;
  }

  const auto req = std::make_shared<tobas_msgs::srv::ConfigureImuLowPassFilter::Request>();
  req->accel_cutoff = lowpass_cfg_.accel_cutoff;
  req->gyro_cutoff = lowpass_cfg_.gyro_cutoff;
  req->dgyro_cutoff = lowpass_cfg_.dgyro_cutoff;

  config_lowpass_filter_sc_->async_send_request(req);

  return true;
}

bool ImuFilterConfigServer::sendRpmFilterConfigRequest()
{
  if (!config_rpm_filter_sc_->service_is_ready()) {
    TOBAS_ERROR("\"", service::kConfigureImuRpmFilter, "\" is not ready.");
    return false;
  }

  const auto req = std::make_shared<tobas_msgs::srv::ConfigureImuRpmFilter::Request>();
  req->quality_factor = notch_cfg_.quality_factor;
  req->min_center_freq = notch_cfg_.min_center_freq;
  req->fade_range = notch_cfg_.fade_range;
  req->lpf_cutoff = notch_cfg_.lpf_cutoff;

  config_rpm_filter_sc_->async_send_request(req);

  return true;
}

bool ImuFilterConfigServer::lowPassFilterAccelCutoffCb(const long& p)
{
  lowpass_cfg_.accel_cutoff = p;

  if (lowPassFilterConfigReady()) {
    if (!sendLowPassFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::lowPassFilterGyroCutoffCb(const long& p)
{
  lowpass_cfg_.gyro_cutoff = p;

  if (lowPassFilterConfigReady()) {
    if (!sendLowPassFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::lowPassFilterDGyroCutoffCb(const long& p)
{
  lowpass_cfg_.dgyro_cutoff = p;

  if (lowPassFilterConfigReady()) {
    if (!sendLowPassFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::rpmFilterQualityFactorCb(const long& p)
{
  notch_cfg_.quality_factor = p;

  if (notchFilterConfigReady()) {
    if (!sendRpmFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::rpmFilterMinCenterFreqCb(const long& p)
{
  notch_cfg_.min_center_freq = p;

  if (notchFilterConfigReady()) {
    if (!sendRpmFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::rpmFilterFadeRangeCb(const long& p)
{
  notch_cfg_.fade_range = p;

  if (notchFilterConfigReady()) {
    if (!sendRpmFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::rpmFilterLpfCutoffCb(const long& p)
{
  notch_cfg_.lpf_cutoff = p;

  if (notchFilterConfigReady()) {
    if (!sendRpmFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

void ImuFilterConfigServer::imuRawCb(const tobas_msgs::Imu::ConstSharedPtr&)
{
  // Register dynamic parameters after confirming that raw IMU data can be received, meaning the node managing the IMU
  // filter is running. This ensures that the initial filter settings are applied reliably.

  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_ACCEL_CUTOFF
  addDynamicIntParam("lowpass_filter/accel_cutoff", &self::lowPassFilterAccelCutoffCb, this, 5, 6, 0, 20, " Hz");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_GYRO_CUTOFF
  addDynamicIntParam("lowpass_filter/gyro_cutoff", &self::lowPassFilterGyroCutoffCb, this, 5, 8, 0, 20, " Hz");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_DGYRO_CUTOFF
  addDynamicIntParam("lowpass_filter/dgyro_cutoff", &self::lowPassFilterDGyroCutoffCb, this, 5, 4, 0, 20, " Hz");

  // cf. https://betaflight.com/docs/wiki/guides/current/DSHOT-RPM-Filtering
  if (config_rpm_filter_sc_) {
    addDynamicIntParam("rpm_filter/quality_factor", &self::rpmFilterQualityFactorCb, this, 1, 0, 0, 10);  // Disabled
    addDynamicIntParam("rpm_filter/min_center_frequency", &self::rpmFilterMinCenterFreqCb, this, 10, 10, 2, 20, " Hz");
    addDynamicIntParam("rpm_filter/fade_range", &self::rpmFilterFadeRangeCb, this, 5, 10, 0, 20, " Hz");
    addDynamicIntParam("rpm_filter/lpf_cutoff", &self::rpmFilterLpfCutoffCb, this, 10, 10, 1, 20, " Hz");
  }

  // Cancel subscription.
  imu_raw_sub_.reset();
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::ImuFilterConfigServer)
