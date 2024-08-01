#include <tobas_tools/constants.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_msgs/Throttle.h>

#include "../include/tobas_gazebo_ros/rotor_command_handler.hpp"
#include "../include/tobas_gazebo_ros/constants.hpp"

using namespace std;

namespace tobas_gazebo_ros
{
RotorCommandHandler::RotorCommandHandler(, const string& name)
  : super(node, pnh, name)
{
  drone_.loadFromParam(node_);

  for (const auto& rotor : drone_.rotorConfigs())
  {
    const auto topic = string(gazebo::kThrottleTopicPrefix) + "_" + to_string(rotor.channel);
    throttle_pubs_[rotor.channel] = node_.advertise<tobas_gazebo_msgs::Throttle>(topic, 1);
  }

  throttles_sub_ = node_.subscribe(tobas::kThrottlesCmdTopic, 1, &self::throttlesCb, this, tcpNoDelay());
  enable_rcout_srv_ = node_.advertiseService(tobas::kEnableRcOutputSrv, &self::enableRCOutputCb, this);
}

void RotorCommandHandler::throttlesCb(const tobas_msgs::ThrottleArrayConstPtr& throttles)
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
    const auto out = boost::make_shared<tobas_gazebo_msgs::Throttle>();
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
