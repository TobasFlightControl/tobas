#pragma once

#include <cstddef>
#include <cmath>

namespace dsp
{
/**
 * @brief Welfordのアルゴリズムで逐次的に平均と分散を計算する．
 * cf. https://blog.data-hacker.net/2020/11/welford.html
 *
 * @note 数値誤差は小さいが，分散とデータ数の積を保持するためデータ数が大きすぎると発散する．
 */
class Welford
{
public:
  explicit Welford();

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
  size_t n_;      // データ数
  double mean_;   // 平均
  double var_n_;  // 分散とデータ数の積
};

inline double Welford::getMean() const
{
  return mean_;
}

inline double Welford::getVariance() const
{
  return n_ > 0 ? var_n_ / n_ : 0.;
}

inline double Welford::getStddev() const
{
  return sqrt(getVariance());
}

inline size_t Welford::getCount() const
{
  return n_;
}
}  // namespace dsp
