#include <tobas_math/core.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_a1_ros/dshot_driver.hpp"

using namespace std;

namespace a1
{
DShotDriver::DShotDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) : super(nh, pnh, name)
{
  if (!dshot_.initialize())
    TOBAS_EXIT("Failed to initialize DSHOT driver.");

  throttles_sub_ = nh_.subscribe(tobas::kThrottlesCmdTopic, 1, &self::throttlesCb, this, tcpNoDelay());
  enable_rcout_srv_ = nh_.advertiseService(tobas::kEnableRcOutputSrv, &self::enableRCOutputCb, this);
}

void DShotDriver::throttlesCb(const tobas_msgs::ThrottleArrayConstPtr& throttles)
{
  // Set throttles of each channel
  for (const auto& elem : throttles->throttles)
  {
    if (elem.channel >= DShot::kChannelSize)
    {
      TOBAS_ERROR("DSHOT channel ", elem.channel, " does not exist.");
      continue;
    }

    if (!is_enabled_.at(elem.channel))
    {
      TOBAS_ERROR("DSHOT channel ", elem.channel, " is disabled.");
      continue;
    }

    if (elem.throttle < tobas::kMinThrottle)
    {
      TOBAS_ASSERT(dshot_.setThrottle(elem.channel, DShot::DISARM));
    }
    else
    {
      auto throttle = static_cast<uint16_t>(math::remap<double>(
        elem.throttle, tobas::kMinThrottle, tobas::kMaxThrottle, DShot::kMinThrottle, DShot::kMaxThrottle));
      throttle = clamp(throttle, DShot::kMinThrottle, DShot::kMaxThrottle);
      TOBAS_ASSERT(dshot_.setThrottle(elem.channel, throttle));
    }
  }

  // Send DSHOT throttles
  if (!dshot_.transfer())
    TOBAS_ERROR("Failed to send DSHOT command.");
}

bool DShotDriver::enableRCOutputCb(tobas_msgs::EnableRCOutputRequest& req, tobas_msgs::EnableRCOutputResponse& res)
{
  if (req.channel >= DShot::kChannelSize)
  {
    res.success = false;
    res.message = "DSHOT channel out of range.";
    return true;
  }

  if (req.enable)
  {
    is_enabled_.at(req.channel) = true;
  }
  else
  {
    TOBAS_ASSERT(dshot_.setDisabled(req.channel));
    is_enabled_.at(req.channel) = false;
  }

  res.success = true;
  res.message = "";
  return true;
}
}  // namespace a1
