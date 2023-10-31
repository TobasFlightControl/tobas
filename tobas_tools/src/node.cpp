#include <dh_ros_tools/console_message.hpp>

#include "../include/tobas_tools/node.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;

namespace tobas
{
BaseNode::BaseNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : nh_(nh), pnh_(pnh), name_(name)
{
}

void BaseNode::registerSubscribers()
{
  event_sub_ = nh_.subscribe(tobas::kEventTopic, 1, &BaseNode::eventCb, this, tcpNoDelay());
}

bool BaseNode::updateCommandLevel(uint8_t& cur_level, const uint8_t& new_level)
{
  if (new_level < cur_level)
  {
    rosErrorThrottle(
      kCommandLevelErrorPeriod, name_,
      "The command is ignored because its level " << static_cast<int>(new_level)
                                                  << "is lower than the current command level "
                                                  << static_cast<int>(cur_level) << ".");
    return false;
  }

  if (new_level > cur_level)
  {
    rosInfo(
      name_, "The command level is raised from " << static_cast<int>(cur_level) << " to "
                                                 << static_cast<int>(new_level) << ".");
    cur_level = new_level;
  }

  return true;
}
}  // namespace tobas
