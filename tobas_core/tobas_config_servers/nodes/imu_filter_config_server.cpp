#include <tobas_constants/node.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/srv/configure_imu_filter.hpp>
#include <tobas_msgs_adapter/imu.hpp>

namespace tobas
{
class ImuFilterConfigServer : public tobas::BaseNode
{
  using self = ImuFilterConfigServer;
  using super = tobas::BaseNode;

public:
  explicit ImuFilterConfigServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  int accel_cutoff_ = -1;
  int gyro_cutoff_ = -1;
  int dgyro_cutoff_ = -1;

  ros2::SubscriberPtr<tobas_msgs::Imu> imu_raw_sub_;
  ros2::ServiceClientPtr<tobas_msgs::srv::ConfigureImuFilter> config_sc_;

  bool imuConfigReady() const;
  bool sendImuConfigRequest();

  void imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& msg);

  bool accelLowPassCutoffCb(const long& p);
  bool gyroLowPassCutoffCb(const long& p);
  bool dGyroLowPassCutoffCb(const long& p);
};

ImuFilterConfigServer::ImuFilterConfigServer(const rclcpp::NodeOptions& options)
  : super(tobas::node::kImuFilterConfigServer, nodeOptions_DParam(options))
{
  imu_raw_sub_ = createSubscriber(tobas::topic::kImuRaw, &self::imuRawCb, this);
}

bool ImuFilterConfigServer::imuConfigReady() const
{
  return accel_cutoff_ >= 0 && gyro_cutoff_ >= 0 && dgyro_cutoff_ >= 0;
}

bool ImuFilterConfigServer::sendImuConfigRequest()
{
  if (!config_sc_->service_is_ready()) {
    TOBAS_ERROR("\"", tobas::service::kConfigureImuFilter, "\" is not ready.");
    return false;
  }

  const auto req = std::make_shared<tobas_msgs::srv::ConfigureImuFilter::Request>();
  req->accel_cutoff = accel_cutoff_;
  req->gyro_cutoff = gyro_cutoff_;
  req->dgyro_cutoff = dgyro_cutoff_;

  config_sc_->async_send_request(req);

  return true;
}

void ImuFilterConfigServer::imuRawCb(const tobas_msgs::Imu::ConstSharedPtr&)
{
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_ACCEL_CUTOFF
  addDynamicIntParam("accel_lowpass_cutoff", &self::accelLowPassCutoffCb, this, 30, 1, 100, " Hz");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_GYRO_CUTOFF
  addDynamicIntParam("gyro_lowpass_cutoff", &self::gyroLowPassCutoffCb, this, 40, 1, 100, " Hz");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#IMU_DGYRO_CUTOFF
  addDynamicIntParam("dgyro_lowpass_cutoff", &self::dGyroLowPassCutoffCb, this, 20, 1, 100, " Hz");

  config_sc_ = create_client<tobas_msgs::srv::ConfigureImuFilter>(tobas::service::kConfigureImuFilter);

  // Cancel subscription
  imu_raw_sub_.reset();
}

bool ImuFilterConfigServer::accelLowPassCutoffCb(const long& p)
{
  accel_cutoff_ = p;

  if (imuConfigReady()) {
    if (!sendImuConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::gyroLowPassCutoffCb(const long& p)
{
  gyro_cutoff_ = p;

  if (imuConfigReady()) {
    if (!sendImuConfigRequest()) {
      return false;
    }
  }

  return true;
}

bool ImuFilterConfigServer::dGyroLowPassCutoffCb(const long& p)
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
