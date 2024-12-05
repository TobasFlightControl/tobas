#pragma once

#include <tobas_eigen_tools/core.hpp>

namespace dsp
{
/**
 * @brief Welfordのアルゴリズムで逐次的に平均と分散を計算する (memo: 2-65)
 *
 * @note 数値誤差は小さいが，分散とデータ数の積を保持するためデータ数が大きすぎると発散する．
 */
template <typename Scalar, int Size>
class Welford
{
  using DataType = Eigen::Vector<Scalar, Size>;
  using CovType = Eigen::Matrix<Scalar, Size, Size>;

public:
  explicit Welford();

  /* 内部変数をリセットする． */
  void reset();

  /* 新しいデータを追加する． */
  inline void add(const DataType& x);

  /* 平均を取得する． */
  inline const DataType& mean() const;

  /* 分散を取得する． */
  inline CovType variance() const;

  /* データの数を取得する． */
  inline size_t count() const;

private:
  size_t n_;       // データ数
  DataType mean_;  // 平均
  CovType var_n_;  // 分散とデータ数の積
};

template <typename Scalar, int Size>
Welford<Scalar, Size>::Welford()
{
  static_assert(Size > 0);
  reset();
}

template <typename Scalar, int Size>
void Welford<Scalar, Size>::reset()
{
  n_ = 0;
  mean_.setZero();
  var_n_.setZero();
}

template <typename Scalar, int Size>
inline void Welford<Scalar, Size>::add(const DataType& x)
{
  ++n_;

  const DataType d = x - mean_;
  mean_ += d / n_;

  const DataType d2 = x - mean_;
  var_n_ += d * d2.transpose();
  eigen::symmetrise(var_n_);
}

template <typename Scalar, int Size>
inline const Welford<Scalar, Size>::DataType& Welford<Scalar, Size>::mean() const
{
  return mean_;
}

template <typename Scalar, int Size>
inline Welford<Scalar, Size>::CovType Welford<Scalar, Size>::variance() const
{
  if (n_ > 0)
    return var_n_ / n_;
  else
    return CovType::Zero();
}

template <typename Scalar, int Size>
inline size_t Welford<Scalar, Size>::count() const
{
  return n_;
}
}  // namespace dsp
