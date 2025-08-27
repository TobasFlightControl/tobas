#pragma once

#include <tobas_msgs_adapter/link_state_array.hpp>

namespace manipulation
{
std::vector<std::string> linkNames(const tobas_msgs::LinkStateArray& msg);
}  // namespace manipulation
