#include <cassert>

#include "../include/tobas_gazebo_plugins/rate_manager.hpp"

#define NANO_1SEC 1'000'000'000

using namespace std;

namespace gazebo
{
RateManager::RateManager(const size_t& update_rate) : update_rate_(update_rate), t_next_(0ns)
{
  assert(update_rate > 0);
  assert(update_rate < NANO_1SEC);
}

bool RateManager::update(const chrono::steady_clock::duration& time)
{
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
