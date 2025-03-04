#include <tobas_property_tree/property_tree.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_msgs_adapter/imu_stamped.hpp>
#include <tobas_real_msgs/srv/set_imu_params.hpp>

using namespace std;
using namespace real::handler::imu;
namespace fs = filesystem;

class ImuHandlerNode : public tobas::BaseNode
{
  using self = ImuHandlerNode;
  using super = tobas::BaseNode;
  using SetParams = tobas_real_msgs::srv::SetIMUParams;

public:
  explicit ImuHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  kdl::Vector acc_bias_;  // [m/s^2]

  ptree::PropertyTree pt_;

  ros2::PublisherPtr<tobas_msgs::ImuStamped> imu_pub_;
  ros2::SubscriberPtr<tobas_msgs::ImuStamped> imu_sub_;
  ros2::ServiceServerPtr<SetParams> set_params_ss_;

  bool getConfig();
  void registerPubSub();

  void imuCb(const tobas_msgs::ImuStamped::ConstSharedPtr& imu_in);
  void setParamsCb(const SetParams::Request::ConstSharedPtr& req, const SetParams::Response::SharedPtr& res);
};

ImuHandlerNode::ImuHandlerNode(const rclcpp::NodeOptions& options) : super("real_imu_handler", options)
{
  if (!pt_.initialize((fs::path(tobas::kConfigDirRoot) / get_name()).replace_extension(".ini")))
  {
    TOBAS_ERROR("Failed to initialize property tree. This node will not work.");
    return;
  }

  set_params_ss_ = createService<SetParams>(kSetParamSrv, &self::setParamsCb, this);

  if (!getConfig())
  {
    TOBAS_ERROR("Failed to get configurations. This node will not work until they are set.");
    return;
  }

  imu_pub_ = createPublisher<tobas_msgs::ImuStamped>(tobas::kImuRawTopic);
  imu_sub_ = createSubscriber(real::kIMUTopic, &self::imuCb, this);
}

bool ImuHandlerNode::getConfig()
{
  if (!pt_.get(kOffsetXKey, acc_bias_.x()))
  {
    TOBAS_ERROR("Failed to get \"", kOffsetXKey, "\".");
    return false;
  }

  if (!pt_.get(kOffsetYKey, acc_bias_.y()))
  {
    TOBAS_ERROR("Failed to get \"", kOffsetXKey, "\".");
    return false;
  }

  if (!pt_.get(kOffsetZKey, acc_bias_.z()))
  {
    TOBAS_ERROR("Failed to get \"", kOffsetXKey, "\".");
    return false;
  }

  return true;
}

void ImuHandlerNode::registerPubSub()
{
  imu_pub_ = createPublisher<tobas_msgs::ImuStamped>(tobas::kImuRawTopic);
  imu_sub_ = createSubscriber(real::kIMUTopic, &self::imuCb, this);
}

void ImuHandlerNode::imuCb(const tobas_msgs::ImuStamped::ConstSharedPtr& imu_in)
{
  auto imu_out = std::make_unique<tobas_msgs::ImuStamped>(*imu_in);
  imu_out->imu.accel -= acc_bias_;  // Remove accel bias
  imu_pub_->publish(move(imu_out));
}

void ImuHandlerNode::setParamsCb(
  const SetParams::Request::ConstSharedPtr& req,
  const SetParams::Response::SharedPtr& res)
{
  // Update parameters
  acc_bias_.x(req->offset_x);
  acc_bias_.y(req->offset_y);
  acc_bias_.z(req->offset_z);

  // Save parameters
  pt_.set(kOffsetXKey, req->offset_x);
  pt_.set(kOffsetYKey, req->offset_y);
  pt_.set(kOffsetZKey, req->offset_z);
  if (!pt_.save())
  {
    res->success = false;
    res->message = "Failed to save parameters.";
    return;
  }

  if (imu_pub_ == nullptr)
    registerPubSub();

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuHandlerNode)
