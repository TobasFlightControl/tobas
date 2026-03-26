#include "tobas_manipulation/util.hpp"

namespace tobas
{
namespace manipulation
{
std::vector<std::string> linkNames(const tobas_msgs::LinkStateArray& msg)
{
  std::vector<std::string> res;
  for (const auto& state : msg.states) {
    res.push_back(state.name);
  }
  return res;
}
}  // namespace manipulation
}  // namespace tobas
