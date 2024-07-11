#include <tobas_std_tools/map.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_msgs/Throttle.h>

#include "../include/tobas_gazebo_ros/rotor_command_handler.hpp"
#include "../include/tobas_gazebo_ros/constants.hpp"

using namespace std;

namespace tobas_gazebo_ros
{
RotorCommandHandler::RotorCommandHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  drone_.loadFromParam(nh_);

  for (const auto& rotor : drone_.rotorConfigs())
  {
    const auto topic = string(gazebo::kThrottleTopicPrefix) + "_" + to_string(rotor.channel);
    throttle_pubs_[rotor.channel] = nh_.advertise<tobas_gazebo_msgs::Throttle>(topic, 1);
  }

  throttles_sub_ = nh_.subscribe(tobas::kThrottlesCmdTopic, 1, &self::throttlesCb, this, tcpNoDelay());
  enable_pwm_srv_ = nh_.advertiseService(tobas::kEnablePwmSrv, &self::enablePwmCb, this);
}

void RotorCommandHandler::throttlesCb(const tobas_msgs::ThrottleArrayConstPtr& throttles)
{
  for (const auto& in : throttles->throttles)
  {
    // Check channel
    if (!tobas_std::contains(throttle_pubs_, in.channel))
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

bool RotorCommandHandler::enablePwmCb(tobas_msgs::EnablePwmRequest& req, tobas_msgs::EnablePwmResponse& res)
{
  if (!tobas_std::contains(throttle_pubs_, req.channel))
  {
    TOBAS_ERROR("The drone does not have rotor channel ", req.channel, ".");
    res.success = false;
    return true;
  }

  // TODO: ちゃんとサービスを実装する

  res.success = true;
  return true;
}
}  // namespace tobas_gazebo_ros
