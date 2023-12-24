#pragma once

#include <cstddef>

namespace dh_std
{
/**
 * @brief Welfordのアルゴリズムで逐次的に統計量を計算する．
 * cf. https://blog.data-hacker.net/2020/11/welford.html
 */
class OnlineStatistics
{
public:
  explicit OnlineStatistics();

  /* 内部変数をリセットする． */
  void reset();

  /* 新しいデータを追加する． */
  void addData(const double& x);

  /* 平均を取得する． */
  double getMean() const;

  /* 分散を取得する． */
  double getVariance() const;

  /* データの数を取得する． */
  size_t getCount() const;

private:
  size_t n_;     // データの数
  double mean_;  // 平均
  double m2_;    // 分散を計算するための中間項
};
}  // namespace dh_std
