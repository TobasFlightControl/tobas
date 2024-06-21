#include <tobas_std_tools/vector.hpp>

#include "../include/tobas_dsp/moving_stat.hpp"

using namespace std;

namespace dsp
{
MovingStatistics::MovingStatistics()
{
}

void MovingStatistics::initialize(const std::vector<double>& data)
{
  size_ = data.size();
  if (size_ == 0)
    throw runtime_error("Data size must be positive.");

  que_.clear();
  for (const auto& x : data)
    que_.push_back(x);

  m_ = tobas_std::fmean(data);
  v_ = tobas_std::variance(data);
}

void MovingStatistics::add(double new_x)
{
  const auto old_x = que_.front();

  // 平均・分散を更新
  const auto d = (new_x - old_x) / size_;
  m_ += d;
  v_ += d * (new_x + old_x + d - 2 * m_);

  // キューを更新
  que_.pop_front();
  que_.push_back(new_x);
}
}  // namespace dsp
