// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/node.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/srv/configure_imu_low_pass_filter.hpp>
#include <tobas_msgs/srv/configure_imu_notch_filter.hpp>
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
  // Static parameters
  bool has_rpm_filter_;

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
  } notch_cfg_;

  ros2::SubscriberPtr<tobas_msgs::Imu> imu_raw_sub_;
  ros2::ServiceClientPtr<tobas_msgs::srv::ConfigureImuLowPassFilter> config_lowpass_sc_;
  ros2::ServiceClientPtr<tobas_msgs::srv::ConfigureImuNotchFilter> config_notch_sc_;

  bool lowPassFilterConfigReady() const;
  bool notchFilterConfigReady() const;
  bool sendLowPassFilterConfigRequest();
  bool sendRpmFilterConfigRequest();

  bool accelCutoffCb(const long& p);
  bool gyroCutoffCb(const long& p);
  bool dGyroCutoffCb(const long& p);
  bool qualityFactorCb(const long& p);
  bool minCenterFreqCb(const long& p);
  bool fadeRangeCb(const long& p);

  void imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& msg);
};

ImuFilterConfigServer::ImuFilterConfigServer(const rclcpp::NodeOptions& options)
  : super(node::kImuFilterConfigServer, nodeOptions_DParam(options))
{
  has_rpm_filter_ = getBoolParam("has_rpm_filter");

  imu_raw_sub_ = createSubscriber(topic::kImuRaw, &self::imuRawCb, this);

  config_lowpass_sc_ = create_client<tobas_msgs::srv::ConfigureImuLowPassFilter>(service::kConfigureImuLowPassFilter);
  config_notch_sc_ = create_client<tobas_msgs::srv::ConfigureImuNotchFilter>(service::kConfigureImuRpmFilter);
}

bool ImuFilterConfigServer::lowPassFilterConfigReady() const
{
  return lowpass_cfg_.accel_cutoff >= 0 && lowpass_cfg_.gyro_cutoff >= 0 && lowpass_cfg_.dgyro_cutoff >= 0;
}

bool ImuFilterConfigServer::notchFilterConfigReady() const
{
  return notch_cfg_.quality_factor >= 0 && notch_cfg_.min_center_freq >= 0 && notch_cfg_.fade_range >= 0;
}

bool ImuFilterConfigServer::sendLowPassFilterConfigRequest()
{
  if (!config_lowpass_sc_->service_is_ready()) {
    TOBAS_ERROR("\"", service::kConfigureImuLowPassFilter, "\" is not ready.");
    return false;
  }

  const auto req = std::make_shared<tobas_msgs::srv::ConfigureImuLowPassFilter::Request>();
  req->accel_cutoff = lowpass_cfg_.accel_cutoff;
  req->gyro_cutoff = lowpass_cfg_.gyro_cutoff;
  req->dgyro_cutoff = lowpass_cfg_.dgyro_cutoff;

  config_lowpass_sc_->async_send_request(req);

  return true;
}

bool ImuFilterConfigServer::sendRpmFilterConfigRequest()
{
  if (!config_notch_sc_->service_is_ready()) {
    TOBAS_ERROR("\"", service::kConfigureImuRpmFilter, "\" is not ready.");
    return false;
  }

  const auto req = std::make_shared<tobas_msgs::srv::ConfigureImuNotchFilter::Request>();
  req->quality_factor = notch_cfg_.quality_factor;
  req->min_center_freq = notch_cfg_.min_center_freq;
  req->fade_range = notch_cfg_.fade_range;

  config_notch_sc_->async_send_request(req);

  return true;
}

bool ImuFilterConfigServer::accelCutoffCb(const long& p)
{
  lowpass_cfg_.accel_cutoff = p;

  if (lowPassFilterConfigReady()) {
    if (!sendLowPassFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::gyroCutoffCb(const long& p)
{
  lowpass_cfg_.gyro_cutoff = p;

  if (lowPassFilterConfigReady()) {
    if (!sendLowPassFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::dGyroCutoffCb(const long& p)
{
  lowpass_cfg_.dgyro_cutoff = p;

  if (lowPassFilterConfigReady()) {
    if (!sendLowPassFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::qualityFactorCb(const long& p)
{
  notch_cfg_.quality_factor = p;

  if (notchFilterConfigReady()) {
    if (!sendRpmFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::minCenterFreqCb(const long& p)
{
  notch_cfg_.min_center_freq = p;

  if (notchFilterConfigReady()) {
    if (!sendRpmFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::fadeRangeCb(const long& p)
{
  notch_cfg_.fade_range = p;

  if (notchFilterConfigReady()) {
    if (!sendRpmFilterConfigRequest()) {
      return false;
    }
  }

  return true;
}

void ImuFilterConfigServer::imuRawCb(const tobas_msgs::Imu::ConstSharedPtr&)
{
  // IMUの生データが受信可能即ちIMUフィルタを管理しているノードが立ち上がっているのを確認してから動的パラメータを登録する．
  // そうすることでLPFカットオフの初期値が確実に反映される．

  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_ACCEL_CUTOFF
  addDynamicIntParam("lowpass_filter/accel_cutoff", &self::accelCutoffCb, this, 30, 0, 100, " Hz");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_GYRO_CUTOFF
  addDynamicIntParam("lowpass_filter/gyro_cutoff", &self::gyroCutoffCb, this, 40, 0, 100, " Hz");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_DGYRO_CUTOFF
  addDynamicIntParam("lowpass_filter/dgyro_cutoff", &self::dGyroCutoffCb, this, 20, 0, 100, " Hz");

  // cf. https://betaflight.com/docs/wiki/guides/current/DSHOT-RPM-Filtering
  if (has_rpm_filter_) {
    addDynamicIntParam("rpm_filter/quality_factor", &self::qualityFactorCb, this, 0, 0, 10);
    addDynamicIntParam("rpm_filter/min_center_frequency", &self::minCenterFreqCb, this, 100, 0, 200, " Hz");
    addDynamicIntParam("rpm_filter/fade_range", &self::fadeRangeCb, this, 50, 0, 100, " Hz");
  }

  // Cancel subscription
  imu_raw_sub_.reset();
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::ImuFilterConfigServer)
