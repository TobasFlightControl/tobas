#include <tobas_constants/constants.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_math/ellipse_transformer.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_ros2_tools/util.hpp>

#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>

using namespace std;
using namespace real::handler::mag;
namespace fs = filesystem;

class MagnetometerHandlerNode : public tobas::BaseNode
{
  using self = MagnetometerHandlerNode;
  using super = tobas::BaseNode;
  using SetParams = tobas_real_msgs::srv::SetMagnetometerParams;

public:
  explicit MagnetometerHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  math::EllipseTransformer mag_trans_;

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

  if (!mag_trans_.initialize()) {
    TOBAS_ERROR("Failed to initialize ellipse transformer. This node will not work.");
    return;
  }

  registerPubSub();
}

bool MagnetometerHandlerNode::getConfig()
{
  if (!pt_.get(ns(), kAxxKey, mag_trans_.a_xx)) {
    TOBAS_ERROR("Failed to get \"", kAxxKey, "\" from configuration file.");
    return false;
  }
  if (!pt_.get(ns(), kAyyKey, mag_trans_.a_yy)) {
    TOBAS_ERROR("Failed to get \"", kAyyKey, "\" from configuration file.");
    return false;
  }
  if (!pt_.get(ns(), kAzzKey, mag_trans_.a_zz)) {
    TOBAS_ERROR("Failed to get \"", kAzzKey, "\" from configuration file.");
    return false;
  }
  if (!pt_.get(ns(), kAxyKey, mag_trans_.a_xy)) {
    TOBAS_ERROR("Failed to get \"", kAxyKey, "\" from configuration file.");
    return false;
  }
  if (!pt_.get(ns(), kAyzKey, mag_trans_.a_yz)) {
    TOBAS_ERROR("Failed to get \"", kAyzKey, "\" from configuration file.");
    return false;
  }
  if (!pt_.get(ns(), kAzxKey, mag_trans_.a_zx)) {
    TOBAS_ERROR("Failed to get \"", kAzxKey, "\" from configuration file.");
    return false;
  }
  if (!pt_.get(ns(), kBxKey, mag_trans_.b_x)) {
    TOBAS_ERROR("Failed to get \"", kBxKey, "\" from configuration file.");
    return false;
  }
  if (!pt_.get(ns(), kByKey, mag_trans_.b_y)) {
    TOBAS_ERROR("Failed to get \"", kByKey, "\" from configuration file.");
    return false;
  }
  if (!pt_.get(ns(), kBzKey, mag_trans_.b_z)) {
    TOBAS_ERROR("Failed to get \"", kBzKey, "\" from configuration file.");
    return false;
  }
  if (!pt_.get(ns(), kCKey, mag_trans_.c)) {
    TOBAS_ERROR("Failed to get \"", kCKey, "\" from configuration file.");
    return false;
  }

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
  mag_out->mag.data = mag_trans_.transform(mag_in->mag.data);  // Project data to unit sphere
  mag_pub_->publish(move(mag_out));
}

void MagnetometerHandlerNode::setParamsCb(
  const SetParams::Request::ConstSharedPtr& req,
  const SetParams::Response::SharedPtr& res)
{
  // Copy transformer
  const auto mag_trans_old = mag_trans_;

  // Update parameters
  mag_trans_.a_xx = req->a_xx;
  mag_trans_.a_yy = req->a_yy;
  mag_trans_.a_zz = req->a_zz;
  mag_trans_.a_xy = req->a_xy;
  mag_trans_.a_yz = req->a_yz;
  mag_trans_.a_zx = req->a_zx;
  mag_trans_.b_x = req->b_x;
  mag_trans_.b_y = req->b_y;
  mag_trans_.b_z = req->b_z;
  mag_trans_.c = req->c;

  // Verify parameters
  if (!mag_trans_.initialize()) {
    res->success = false;
    res->message = "Failed to initialize ellipse transformer.";
    mag_trans_ = mag_trans_old;
    return;
  }

  // Save parameters
  pt_.set(ns(), kAxxKey, req->a_xx);
  pt_.set(ns(), kAyyKey, req->a_yy);
  pt_.set(ns(), kAzzKey, req->a_zz);
  pt_.set(ns(), kAxyKey, req->a_xy);
  pt_.set(ns(), kAyzKey, req->a_yz);
  pt_.set(ns(), kAzxKey, req->a_zx);
  pt_.set(ns(), kBxKey, req->b_x);
  pt_.set(ns(), kByKey, req->b_y);
  pt_.set(ns(), kBzKey, req->b_z);
  pt_.set(ns(), kCKey, req->c);
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
