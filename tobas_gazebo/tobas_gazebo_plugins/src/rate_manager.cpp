#include <cassert>

#include "../include/tobas_gazebo_plugins/rate_manager.hpp"

#define NANO_1SEC 1'000'000'000

using namespace std;

namespace gazebo
{
RateManager::RateManager(const size_t& update_rate) : update_rate_(update_rate), t_next_(0ns)
{
}

bool RateManager::update(const chrono::steady_clock::duration& time)
{
  if (update_rate_ == 0)
    return true;

  if (time < t_next_)
  {
    return false;
  }
  else
  {
    t_next_ += chrono::nanoseconds(NANO_1SEC / update_rate_);
    return true;
  }
}
}  // namespace gazebo
