#pragma once

#include <vector>
#include <Eigen/Core>

namespace eigen_tools
{
/**
 * @brief ブロック対角行列を作る．
 *
 * @param A 部分行列
 * @param num ブロックの個数
 *
 * @return Eigen::MatrixXd
 */
template <typename Derived>
Eigen::MatrixXd blockDiag(const Eigen::MatrixBase<Derived>& A, const Eigen::Index& num)
{
  assert(num > 0);

  const auto r = A.rows();
  const auto c = A.cols();

  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(r * num, c * num);
  for (Eigen::Index i = 0; i < num; ++i)
    res.block(r * i, c * i, r, c) = A;

  return res;
}

/**
 * @brief Aを行または列方向に繰り返してできる行列を返す．
 * cf. numpy.tile()
 *
 * @param A 部分行列
 * @param num 繰り返す回数
 * @param axis 繰り返す方向: Row(0) or Column(1)
 *
 * @return Eigen::MatrixXd
 */
template <typename Derived>
Eigen::MatrixXd tile(const Eigen::MatrixBase<Derived>& A, const Eigen::Index& num, const uint8_t& axis)
{
  assert(num > 0);

  const auto r = A.rows();
  const auto c = A.cols();

  switch (axis)
  {
    case 0:
    {
      Eigen::MatrixXd res(r * num, c);
      for (Eigen::Index i = 0; i < num; ++i)
        res.block(r * i, 0, r, c) = A;
      return res;
    }
    case 1:
    {
      Eigen::MatrixXd res(r, c * num);
      for (Eigen::Index i = 0; i < num; ++i)
        res.block(0, c * i, r, c) = A;
      return res;
    }
    default:
    {
      throw std::runtime_error("axis must be 0 or 1");
    }
  }
}

/**
 * @brief 2つの行列を行方向または列方向に結合する．
 * cf. numpy.concatenate()
 *
 * @param A,B 結合する行列
 * @param axis 結合する方向: Row(0) or Column(1)
 *
 * @return Eigen::MatrixXd
 */
template <typename T, typename U>
Eigen::MatrixXd concat(const Eigen::MatrixBase<T>& A, const Eigen::MatrixBase<U>& B, const uint8_t& axis)
{
  switch (axis)
  {
    case 0:
    {
      assert(A.cols() == B.cols());
      Eigen::MatrixXd res(A.rows() + B.rows(), A.cols());
      res << A, B;
      return res;
    }
    case 1:
    {
      assert(A.rows() == B.rows());
      Eigen::MatrixXd res(A.rows(), A.cols() + B.cols());
      res << A, B;
      return res;
    }
    default:
    {
      throw std::runtime_error("axis must be 0 or 1");
    }
  }
}

/**
 * @brief 3つの行列を行方向または列方向に結合する．
 * cf. numpy.concatenate()
 *
 * @param A,B,C 結合する行列
 * @param axis 結合する方向: Row(0) or Column(1)
 *
 * @return Eigen::MatrixXd
 */
template <typename T, typename U, typename V>
inline Eigen::MatrixXd
concat(const Eigen::MatrixBase<T>& A, const Eigen::MatrixBase<U>& B, const Eigen::MatrixBase<V>& C, const uint8_t& axis)
{
  return concat(concat(A, B, axis), C, axis);
}

/* Eigen::Vectorをstd::vectorに変換する． */
template <typename T, int Rows>
inline std::vector<T> toStdVector(const Eigen::Matrix<T, Rows, 1>& v)
{
  return std::vector<T>(v.data(), v.data() + v.size());
}

/* 行列が正方の場合にtrueを返す． */
template <typename Derived>
inline bool isSquare(const Eigen::MatrixBase<Derived>& A)
{
  return A.rows() == A.cols();
}

/* trueの場合はnan, infが含まれないことを保証する． */
template <typename Derived>
inline bool isFinite(const Eigen::MatrixBase<Derived>& x)
{
  return ((x - x).array() == (x - x).array()).all();
}

/* 正方行列が対称行列かどうかを判定する． */
template <typename Derived>
inline bool isSymmetric(const Eigen::MatrixBase<Derived>& A)
{
  assert(isSquare(A));
  return A.isApprox(A.transpose());
}

/**
 * @brief 正方行列を対称化する．
 *
 * @tparam Derived
 * @param A 対称化する行列
 * @param method 0: 元の行列とその転置の和を2で割る, 1: 下三角部分を上三角部分にコピー
 */
template <typename Derived>
void symmetrise(Eigen::MatrixBase<Derived>& A, const uint8_t& method = 0)
{
  assert(isSquare(A));

  switch (method)
  {
    case 0:
      A = (A + A.transpose()) / 2;
      break;
    case 1:
      for (Eigen::Index i = 0; i < A.rows(); ++i)
        for (Eigen::Index j = i + 1; j < A.cols(); ++j)
          A(i, j) = A(j, i);
      break;
    default:
      throw std::runtime_error("Invalid method: " + std::to_string(method));
  }
}

/* 行方向に平均を計算する． */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Rows, 1> meanRow(const Eigen::Matrix<Scalar, Rows, Cols>& A)
{
  Eigen::Matrix<Scalar, Rows, 1> res;
  for (Eigen::Index r = 0; r < Rows; ++r)
    res(r) = A.row(r).mean();
  return res;
}

/* 列方向に平均を計算する． */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Cols, 1> meanCol(const Eigen::Matrix<Scalar, Rows, Cols>& A)
{
  Eigen::Matrix<Scalar, Cols, 1> res;
  for (Eigen::Index c = 0; c < Cols; ++c)
    res(c) = A.col(c).mean();
  return res;
}

/* 行方向に分散を計算する． */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Rows, 1> varianceRow(const Eigen::Matrix<Scalar, Rows, Cols>& A)
{
  const Eigen::Matrix<Scalar, Rows, 1> mean = meanRow(A);
  Eigen::Matrix<Scalar, Rows, 1> res;
  for (Eigen::Index r = 0; r < Rows; ++r)
    res(r) = (A.row(r).array() - mean(r)).square().mean();
  return res;
}

/* 列方向に分散を計算する． */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Cols, 1> varianceCol(const Eigen::Matrix<Scalar, Rows, Cols>& A)
{
  const Eigen::Matrix<Scalar, Cols, 1> mean = meanCol(A);
  Eigen::Matrix<Scalar, Cols, 1> res;
  for (Eigen::Index c = 0; c < Cols; ++c)
    res(c) = (A.col(c).array() - mean(c)).square().mean();
  return res;
}

/* ベクトルの分散を計算する． */
template <typename Scalar, int Rows>
inline Scalar variance(const Eigen::Matrix<Scalar, Rows, 1>& A)
{
  return varianceCol(A)(0);
}
}  // namespace eigen_tools
