#pragma once

#include <chrono>
#include <memory>

namespace gazebo
{
class RateManager
{
public:
  using SharedPtr = std::shared_ptr<RateManager>;

  explicit RateManager(const size_t& update_rate);

  /* 実行可能な周期ならばtrue． */
  bool update(const std::chrono::steady_clock::duration& cur_time);

private:
  const size_t update_rate_;
  std::chrono::steady_clock::duration next_time_;
};
}  // namespace gazebo
