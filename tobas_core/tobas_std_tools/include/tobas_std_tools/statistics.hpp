#pragma once

#include <cstddef>
#include <cmath>

namespace tobas_std
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
  inline double getMean() const;

  /* 分散を取得する． */
  inline double getVariance() const;

  /* 標準偏差を取得する． */
  inline double getStddev() const;

  /* データの数を取得する． */
  inline size_t getCount() const;

private:
  size_t n_;     // データの数
  double mean_;  // 平均
  double m2_;    // 分散を計算するための中間項
};

inline double OnlineStatistics::getMean() const
{
  return mean_;
}

inline double OnlineStatistics::getVariance() const
{
  // データ数が2未満の場合，分散は定義されていない
  return n_ >= 2 ? m2_ / (n_ - 1) : 0;
}

inline double OnlineStatistics::getStddev() const
{
  return sqrt(getVariance());
}

inline size_t OnlineStatistics::getCount() const
{
  return n_;
}
}  // namespace tobas_std
