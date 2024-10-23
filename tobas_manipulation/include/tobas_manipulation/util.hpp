#pragma once

#include <tobas_msgs_adapter/LinkStateArray.hpp>

namespace manipulation
{
std::vector<std::string> linkNames(const tobas_msgs::LinkStateArray& msg);
}  // namespace manipulation
