#include <tobas_constants/path.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/ellipsoid.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_real_common/handler.hpp>
#include <tobas_ros2_tools/util.hpp>

#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_real_common/ros_interface.hpp>
#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>

using namespace tobas::real::handler::mag;
namespace fs = std::filesystem;

namespace tobas
{
namespace real
{
class MagnetometerHandlerNode : public BaseNode
{
  using self = MagnetometerHandlerNode;
  using super = BaseNode;
  using SetParams = tobas_real_msgs::srv::SetMagnetometerParams;

public:
  explicit MagnetometerHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  eigen::Ellipsoid ellipsoid_;

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
  : super("real_magnetometer_handler", nodeOptions_Default(options))
{
  const auto cfg_dir = linux::isSuperUser() ? fs::path(kConfigDirRoot) : ros2::expandUser(kConfigDirHome);
  if (!pt_.initialize((cfg_dir / kConfigFileName))) {
    TOBAS_ERROR("Failed to initialize property tree. This node will not work.");
    return;
  }

  set_params_ss_ = createService<SetParams>(kSetParamSrv, &self::setParamsCb, this);

  if (!getConfig()) {
    TOBAS_ERROR("Failed to get configuration. This node will not work until they are set.");
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
  mag_pub_ = createPublisher<tobas_msgs::MagneticField>(topic::kMagneticField);
  mag_sub_ = createSubscriber(real::topic::kMagneticField, &self::magCb, this);
}

void MagnetometerHandlerNode::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag_in)
{
  auto mag_out = std::make_unique<tobas_msgs::MagneticField>(*mag_in);
  mag_out->mag.data = ellipsoid_.toUnitSphere(mag_in->mag.data);
  mag_pub_->publish(std::move(mag_out));
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
}  // namespace real
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::real::MagnetometerHandlerNode)
