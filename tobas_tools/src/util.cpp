#include "../include/tobas_tools/util.hpp"

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

namespace tobas
{
std::string addThrotNS(const std::string& topic)
{
  return path::join(kThrottledTopicNS, topic);
}

std::string addIfaceNS(const std::string& topic)
{
  return path::join(kRemoteIfaceTopicNS, topic);
}
}  // namespace tobas
