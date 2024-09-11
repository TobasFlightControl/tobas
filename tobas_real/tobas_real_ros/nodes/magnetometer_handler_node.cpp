#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/ellipse_transformer.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs_adapter/MagneticField.hpp>
#include <tobas_msgs_adapter/MagneticField.hpp>

#include <tobas_real_common/constants.hpp>

using namespace std;

class MagnetometerHandlerNode : public tobas::BaseNode
{
  static constexpr double kHpfCutoff = 10.;  // [Hz] (G(1Hz) ~ 0.1, G(20Hz) ~ 0.9)
  static constexpr size_t kWindowSize = 100;

  using self = MagnetometerHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit MagnetometerHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  math::EllipseTransformer mag_trans_;

  tobas_hal_msgs::MagneticField::ConstSharedPtr mag_raw_;
  ptree::PropertyClient::SharedPtr property_client_;
  std::array<dsp::NoiseVarianceFilter, 3> mag_noise_;

  ros2::PublisherPtr<tobas_msgs::MagneticField> mag_pub_;
  ros2::SubscriberPtr<tobas_hal_msgs::MagneticField> mag_sub_;
  ros2::ServicePtr<std_srvs::srv::Trigger> reload_config_srv_;

  ros2::TimerPtr initialize_timer_;
  void initializeTimerCb();

  bool reloadConfig();

  void magCb(const tobas_hal_msgs::MagneticField::ConstSharedPtr& mag_raw);
  void reloadConfigCb(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr& req,
    const std_srvs::srv::Trigger::Response::SharedPtr& res);
};

MagnetometerHandlerNode::MagnetometerHandlerNode(const rclcpp::NodeOptions& options)
  : super("magnetometer_handler", options)
{
  initialize_timer_ = createTimer(0ns, &self::initializeTimerCb, this);
}

void MagnetometerHandlerNode::initializeTimerCb()
{
  property_client_ = std::make_shared<ptree::PropertyClient>(shared_from_this(), real::kPropertyServerFC);
  reloadConfig();

  mag_pub_ = createPublisher<tobas_msgs::MagneticField>(tobas::kMagTopic);
  mag_sub_ = createSubscriber(hal::kMagTopic, &self::magCb, this);

  reload_config_srv_ =
    createService<std_srvs::srv::Trigger>(name() + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);

  initialize_timer_->cancel();
}

bool MagnetometerHandlerNode::reloadConfig()
{
  if (property_client_->get(real::kConfigKey_MagEllipseAxx, mag_trans_.a_xx) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_->get(real::kConfigKey_MagEllipseAyy, mag_trans_.a_yy) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_->get(real::kConfigKey_MagEllipseAzz, mag_trans_.a_zz) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_->get(real::kConfigKey_MagEllipseAxy, mag_trans_.a_xy) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_->get(real::kConfigKey_MagEllipseAyz, mag_trans_.a_yz) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_->get(real::kConfigKey_MagEllipseAzx, mag_trans_.a_zx) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_->get(real::kConfigKey_MagEllipseBx, mag_trans_.b_x) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_->get(real::kConfigKey_MagEllipseBy, mag_trans_.b_y) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_->get(real::kConfigKey_MagEllipseBz, mag_trans_.b_z) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    mag_trans_.setIdentity();
    return false;
  }
  if (property_client_->get(real::kConfigKey_MagEllipseC, mag_trans_.c) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
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

void MagnetometerHandlerNode::magCb(const tobas_hal_msgs::MagneticField::ConstSharedPtr& mag_raw)
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
    if (mag_noise_[i].update(mag_unit(i), dt) < 0)
      TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Noise filter failed: ", mag_noise_[i].errorMessage());

  // Create message
  auto mag_msg = std::make_unique<tobas_msgs::MagneticField>();

  // Fill header
  mag_msg->header = mag_raw->header;

  // Fill data
  mag_msg->magnetic_field.data = mag_unit;

  // Fill covariance matrices
  mag_msg->covariance.setZero();
  for (size_t i = 0; i < 3; ++i)
    mag_msg->covariance(i, i) = mag_noise_[i].noiseVariance();

  // Publish message
  mag_pub_->publish(move(mag_msg));
}

void MagnetometerHandlerNode::reloadConfigCb(
  const std_srvs::srv::Trigger::Request::ConstSharedPtr&,
  const std_srvs::srv::Trigger::Response::SharedPtr& res)
{
  if (!reloadConfig())
  {
    res->success = false;
    res->message = "Failed to reload configurations.";
    return;
  }

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(MagnetometerHandlerNode)
