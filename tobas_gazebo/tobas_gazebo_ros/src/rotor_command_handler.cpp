#include <tobas_constants/constants.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_msgs/Throttle.h>

#include "../include/tobas_gazebo_ros/rotor_command_handler.hpp"
#include "../include/tobas_gazebo_ros/constants.hpp"

using namespace std;

namespace tobas_gazebo_ros
{
RotorCommandHandler::RotorCommandHandler(const rclcpp::NodeOptions& options)
  : super(node, pnh, name)
{


  for (const auto& rotor : drone_.rotorConfigs())
  {
    const auto topic = string(gazebo::kThrottleTopicPrefix) + "_" + to_string(rotor.channel);
    throttle_pubs_[rotor.channel] = createPublisher<tobas_gazebo_msgs::Throttle>(topic);
  }

  throttles_sub_ = createSubscriber(tobas::kThrottlesCmdTopic, &self::throttlesCb, this);
  enable_rcout_srv_ = createPublisherService(tobas::kEnableRcOutputSrv, &self::enableRCOutputCb, this);
}

void RotorCommandHandler::throttlesCb(const tobas_msgs::ThrottleArray::ConstSharedPtr& throttles)
{
  for (const auto& in : throttles->throttles)
  {
    // Check channel
    if (!throttle_pubs_.contains(in.channel))
    {
      TOBAS_ERROR("The drone does not have rotor channel ", in.channel, ".");
      return;
    }

    // Create throttle message
    const auto out =std::make_unique<tobas_gazebo_msgs::Throttle>();
    out->header = throttles->header;
    out->data = in.throttle;

    // Publish throttle message
    throttle_pubs_.at(in.channel).publish(out);
  }
}

bool RotorCommandHandler::enableRCOutputCb(
  tobas_msgs::EnableRCOutputRequest& req,
  tobas_msgs::EnableRCOutputResponse& res)
{
  if (!throttle_pubs_.contains(req.channel))
  {
    res.success = false;
    res.message = "The drone does not have rotor channel " + to_string(req.channel) + ".";
    return true;
  }

  // TODO: ちゃんとサービスを実装する

  res.success = true;
  res.message = "";
  return true;
}
}  // namespace tobas_gazebo_ros
