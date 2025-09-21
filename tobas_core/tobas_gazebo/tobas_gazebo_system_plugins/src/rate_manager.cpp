#include "tobas_gazebo_system_plugins/rate_manager.hpp"

using namespace std::chrono_literals;
namespace ch = std::chrono;

namespace gazebo
{
RateManager::RateManager(int update_rate) : update_rate_(update_rate), next_time_(0ns)
{
}

bool RateManager::update(const ch::steady_clock::duration& cur_time)
{
  if (update_rate_ <= 0) {
    return true;
  }

  if (cur_time < next_time_) {
    return false;
  }
  else {
    const ch::nanoseconds period(1'000'000'000 / update_rate_);
    if (cur_time - next_time_ < period) {
      next_time_ += period;  // 次回時刻基準で更新することで周波数を守る
    }
    else {
      next_time_ = cur_time + period;  // 2周期以上空いているなら現在時刻基準でリセット
    }
    return true;
  }
}
}  // namespace gazebo
