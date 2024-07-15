#pragma once

#include <tobas_msgs/LinkStateArray.h>

namespace tobas_manipulation
{
std::vector<std::string> linkNames(const tobas_msgs::LinkStateArray& msg);
}  // namespace tobas_manipulation
