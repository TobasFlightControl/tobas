// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/node.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/srv/configure_imu_low_pass_filter.hpp>
#include <tobas_msgs_adapter/imu.hpp>

namespace tobas
{
class ImuFilterConfigServer : public BaseNode
{
  using self = ImuFilterConfigServer;
  using super = BaseNode;

  static constexpr int kMinLpfCutoff = 1;    // [Hz]
  static constexpr int kMaxLpfCutoff = 200;  // [Hz]

public:
  explicit ImuFilterConfigServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Static parameters
  int dflt_accel_cutoff_, dflt_gyro_cutoff_, dflt_dgyro_cutoff_;

  // Dynamic parameters
  int accel_cutoff_ = -1;
  int gyro_cutoff_ = -1;
  int dgyro_cutoff_ = -1;

  ros2::SubscriberPtr<tobas_msgs::Imu> imu_raw_sub_;
  ros2::ServiceClientPtr<tobas_msgs::srv::ConfigureImuLowPassFilter> config_sc_;

  bool imuConfigReady() const;
  bool sendImuConfigRequest();

  void imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& msg);

  bool accelCutoffCb(const long& p);
  bool gyroCutoffCb(const long& p);
  bool dGyroCutoffCb(const long& p);
};

ImuFilterConfigServer::ImuFilterConfigServer(const rclcpp::NodeOptions& options)
  : super(node::kImuFilterConfigServer, nodeOptions_DParam(options))
{
  dflt_accel_cutoff_ = getIntParam("default_accel_lpf_cutoff");
  dflt_gyro_cutoff_ = getIntParam("default_gyro_lpf_cutoff");
  dflt_dgyro_cutoff_ = getIntParam("default_dgyro_lpf_cutoff");

  imu_raw_sub_ = createSubscriber(topic::kImuRaw, &self::imuRawCb, this);
  config_sc_ = create_client<tobas_msgs::srv::ConfigureImuLowPassFilter>(service::kConfigureImuLowPassFilter);
}

bool ImuFilterConfigServer::imuConfigReady() const
{
  return accel_cutoff_ >= 0 && gyro_cutoff_ >= 0 && dgyro_cutoff_ >= 0;
}

bool ImuFilterConfigServer::sendImuConfigRequest()
{
  if (!config_sc_->service_is_ready()) {
    TOBAS_ERROR("\"", service::kConfigureImuLowPassFilter, "\" is not ready.");
    return false;
  }

  const auto req = std::make_shared<tobas_msgs::srv::ConfigureImuLowPassFilter::Request>();
  req->accel_cutoff = accel_cutoff_;
  req->gyro_cutoff = gyro_cutoff_;
  req->dgyro_cutoff = dgyro_cutoff_;

  config_sc_->async_send_request(req);

  return true;
}

void ImuFilterConfigServer::imuRawCb(const tobas_msgs::Imu::ConstSharedPtr&)
{
  // IMUの生データが受信可能即ちIMUフィルタを管理しているノードが立ち上がっているのを確認してから動的パラメータを登録．
  // そうすることでLPFカットオフの初期値が確実に反映される．

  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_ACCEL_CUTOFF
  addDynamicIntParam(
    "accel_lpf_cutoff", &self::accelCutoffCb, this, dflt_accel_cutoff_, kMinLpfCutoff, kMaxLpfCutoff, " Hz");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_GYRO_CUTOFF
  addDynamicIntParam(
    "gyro_lpf_cutoff", &self::gyroCutoffCb, this, dflt_gyro_cutoff_, kMinLpfCutoff, kMaxLpfCutoff, " Hz");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_DGYRO_CUTOFF
  addDynamicIntParam(
    "dgyro_lpf_cutoff", &self::dGyroCutoffCb, this, dflt_dgyro_cutoff_, kMinLpfCutoff, kMaxLpfCutoff, " Hz");

  // Cancel subscription
  imu_raw_sub_.reset();
}

bool ImuFilterConfigServer::accelCutoffCb(const long& p)
{
  accel_cutoff_ = p;

  if (imuConfigReady()) {
    if (!sendImuConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::gyroCutoffCb(const long& p)
{
  gyro_cutoff_ = p;

  if (imuConfigReady()) {
    if (!sendImuConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::dGyroCutoffCb(const long& p)
{
  dgyro_cutoff_ = p;

  if (imuConfigReady()) {
    if (!sendImuConfigRequest()) {
      return false;
    }
  }

  return true;
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::ImuFilterConfigServer)
