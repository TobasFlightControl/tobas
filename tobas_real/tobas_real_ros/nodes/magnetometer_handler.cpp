#include <tobas_constants/constants.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/ellipsoid.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_ros2_tools/util.hpp>

#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>

using namespace real::handler::mag;
namespace fs = std::filesystem;

class MagnetometerHandlerNode : public tobas::BaseNode
{
  using self = MagnetometerHandlerNode;
  using super = tobas::BaseNode;
  using SetParams = tobas_real_msgs::srv::SetMagnetometerParams;

public:
  explicit MagnetometerHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  eigen::Ellipsoid ellipsoid_;

  tobas_msgs::MagneticField::ConstSharedPtr prev_mag_;
  ptree::PropertyTree pt_;

  ros2::PublisherPtr<tobas_msgs::MagneticField> mag_pub_;
  ros2::SubscriberPtr<tobas_msgs::MagneticField> mag_sub_;
  ros2::ServiceServerPtr<SetParams> set_params_ss_;

  bool getConfig();
  void registerPubSub();

  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag_in);
  void setParamsCb(const SetParams::Request::ConstSharedPtr& req, const SetParams::Response::SharedPtr& res);
};

MagnetometerHandlerNode::MagnetometerHandlerNode(const rclcpp::NodeOptions& options)
  : super("real_magnetometer_handler", options)
{
  const auto cfg_dir = linux::isSuperUser() ? fs::path(tobas::kConfigDirRoot) : ros2::expandUser(tobas::kConfigDirHome);
  if (!pt_.initialize((cfg_dir / kConfigFileName))) {
    TOBAS_ERROR("Failed to initialize property tree. This node will not work.");
    return;
  }

  set_params_ss_ = createService<SetParams>(kSetParamSrv, &self::setParamsCb, this);

  if (!getConfig()) {
    TOBAS_ERROR("Failed to get configurations. This node will not work until they are set.");
    return;
  }

  registerPubSub();
}

bool MagnetometerHandlerNode::getConfig()
{
  std::array<double, 3> hard_bias;
  std::array<double, 6> soft_bias;

  if (!pt_.get(ns(), kHardBiasKey, hard_bias)) {
    TOBAS_ERROR("Failed to get \"", kHardBiasKey, "\" from configuration file.");
    return false;
  }
  if (!pt_.get(ns(), kSoftBiasKey, soft_bias)) {
    TOBAS_ERROR("Failed to get \"", kSoftBiasKey, "\" from configuration file.");
    return false;
  }

  ellipsoid_.setHardBias(eigen::fromStdArray(hard_bias));
  ellipsoid_.setSoftBias(eigen::fromStdArray(soft_bias));

  return true;
}

void MagnetometerHandlerNode::registerPubSub()
{
  mag_pub_ = createPublisher<tobas_msgs::MagneticField>(tobas::kMagTopic);
  mag_sub_ = createSubscriber(real::kMagTopic, &self::magCb, this);
}

void MagnetometerHandlerNode::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag_in)
{
  // First message
  if (!prev_mag_) {
    prev_mag_ = mag_in;
    return;
  }

  // Verify that the sensor data is updated
  if (mag_in->mag == prev_mag_->mag) {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Magnetometer data has not been updated—skipping message.");
    return;
  }

  // Update the latest data
  prev_mag_ = mag_in;

  // Publish a calibrated data
  auto mag_out = std::make_unique<tobas_msgs::MagneticField>(*mag_in);
  mag_out->mag.data = ellipsoid_.toUnitSphere(mag_in->mag.data);
  mag_pub_->publish(move(mag_out));
}

void MagnetometerHandlerNode::setParamsCb(
  const SetParams::Request::ConstSharedPtr& req,
  const SetParams::Response::SharedPtr& res)
{
  // Update parameters
  ellipsoid_.setHardBias(eigen::fromStdArray(req->hard_bias));
  ellipsoid_.setSoftBias(eigen::fromStdArray(req->soft_bias));

  // Save parameters
  pt_.set(ns(), kHardBiasKey, req->hard_bias);
  pt_.set(ns(), kSoftBiasKey, req->soft_bias);
  if (!pt_.save()) {
    res->success = false;
    res->message = "Failed to save parameters.";
    return;
  }

  if (!mag_pub_) {
    registerPubSub();
  }

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(MagnetometerHandlerNode)
