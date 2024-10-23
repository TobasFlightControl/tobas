#include "../include/tobas_manipulation/util.hpp"

using namespace std;

namespace manipulation
{
vector<string> linkNames(const tobas_msgs::LinkStateArray& msg)
{
  vector<string> res;
  for (const auto& state : msg.states)
    res.push_back(state.name);
  return res;
}
}  // namespace manipulation
