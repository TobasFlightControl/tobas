#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/throttle_array.hpp>
#include <tobas_msgs/srv/enable_rc_output.hpp>

#include <tobas_aso_core/dshot.hpp>

using namespace std;

class DShotDriverNode : public tobas::BaseNode
{
  using self = DShotDriverNode;
  using super = tobas::BaseNode;
  using EnableSrv = tobas_msgs::srv::EnableRCOutput;

public:
  explicit DShotDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::DShot dshot_;
  std::array<bool, aso::DShot::kChannelSize> is_enabled_;

  SubscriberPtr<tobas_msgs::msg::ThrottleArray> throttles_sub_;
  ServicePtr<EnableSrv> enable_rcout_srv_;

  void throttlesCb(const tobas_msgs::msg::ThrottleArray::ConstSharedPtr& throttles);
  void enableRCOutputCb(const EnableSrv::Request::ConstSharedPtr& req, const EnableSrv::Response::SharedPtr& res);
};

DShotDriverNode::DShotDriverNode(const rclcpp::NodeOptions& options) : super("aso_dshot_driver", options)
{
  if (!dshot_.initialize())
    TOBAS_EXIT("Failed to initialize DSHOT driver.");

  throttles_sub_ = createSubscriber(tobas::kThrottlesCmdTopic, &self::throttlesCb, this);
  enable_rcout_srv_ = createService<EnableSrv>(tobas::kEnableRcOutputSrv, &self::enableRCOutputCb, this);
}

void DShotDriverNode::throttlesCb(const tobas_msgs::msg::ThrottleArray::ConstSharedPtr& throttles)
{
  // Set throttles of each channel
  for (const auto& elem : throttles->throttles)
  {
    if (elem.channel >= aso::DShot::kChannelSize)
    {
      TOBAS_ERROR("DSHOT channel ", elem.channel, " does not exist.");
      continue;
    }

    if (!is_enabled_.at(elem.channel))
    {
      TOBAS_ERROR("DSHOT channel ", elem.channel, " is disabled.");
      continue;
    }

    if (elem.throttle < tobas::kMinThrot)
    {
      TOBAS_ASSERT(dshot_.setThrottle(elem.channel, aso::DShot::DISARM));
    }
    else
    {
      auto throttle = static_cast<uint16_t>(math::remap<double>(
        elem.throttle, tobas::kMinThrot, tobas::kMaxThrot, aso::DShot::kMinThrot, aso::DShot::kMaxThrot));
      throttle = clamp(throttle, aso::DShot::kMinThrot, aso::DShot::kMaxThrot);
      TOBAS_ASSERT(dshot_.setThrottle(elem.channel, throttle));
    }
  }

  // Send DSHOT throttles
  if (!dshot_.transfer())
    TOBAS_ERROR("Failed to send DSHOT command.");
}

void DShotDriverNode::enableRCOutputCb(
  const EnableSrv::Request::ConstSharedPtr& req,
  const EnableSrv::Response::SharedPtr& res)
{
  if (req->channel >= aso::DShot::kChannelSize)
  {
    res->success = false;
    res->message = "DSHOT channel out of range.";
    return;
  }

  if (req->enable)
  {
    is_enabled_.at(req->channel) = true;
  }
  else
  {
    TOBAS_ASSERT(dshot_.setDisabled(req->channel));
    is_enabled_.at(req->channel) = false;
  }

  res->success = true;
  res->message = "";
}

RCLCPP_COMPONENTS_REGISTER_NODE(DShotDriverNode)
