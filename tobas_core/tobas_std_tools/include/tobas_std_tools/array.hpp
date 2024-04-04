#include <array>
#include <cassert>

namespace tobas_std
{
/* 最も近い値のインデックスを返す． */
template <typename T, size_t N>
size_t closestIndex(const std::array<T, N>& arr, const T& a)
{
  assert(N > 0);

  size_t closest_idx = 0;  // コンパイラの警告を防ぐために適当に初期化
  T closest_dist = std::numeric_limits<T>::max();

  for (size_t i = 0; i < N; ++i)
  {
    const T dist = std::abs(arr[i] - a);
    if (dist < closest_dist)
    {
      closest_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}
}  // namespace tobas_std
