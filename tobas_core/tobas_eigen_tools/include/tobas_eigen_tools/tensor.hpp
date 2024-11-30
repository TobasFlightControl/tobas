#pragma once

#include <eigen3/unsupported/Eigen/CXX11/Tensor>

namespace Eigen
{
using Tensor1Xd = Tensor<double, 1>;
using Tensor2Xd = Tensor<double, 2>;
using Tensor3Xd = Tensor<double, 3>;
using Tensor4Xd = Tensor<double, 4>;
}  // namespace Eigen

namespace eigen_tools
{
template <typename TensorType>
inline TensorType shuffle(const TensorType& tensor, const std::initializer_list<int>& order)
{
  static_assert(order.size() == TensorType::NumDimentions);

  Eigen::array<int, TensorType::NumDimentions> shuffle_order;
  std::copy(order.begin(), order.end(), shuffle_order.begin());
  return tensor.shuffle(shuffle_order);
}

template <typename TensorType, typename MatrixType>
inline void setBlock(
  TensorType& _des,
  const Eigen::MatrixBase<MatrixType>& _src,
  const std::initializer_list<int>& _offset,
  const std::initializer_list<int>& _extent)
{
  static_assert(_offset.size() == TensorType::NumDimentions);
  static_assert(_extent.size() == TensorType::NumDimentions);

  Eigen::array<int, TensorType::NumDimentions> offset, extent;
  std::copy(_offset.begin(), _offset.end(), offset.begin());
  std::copy(_extent.begin(), _extent.end(), extent.begin());

  _des.slice(offset, extent) = TensorMap<const TensorType>(_src.data(), _src.size());
}
}  // namespace eigen_tools

template <typename Scalar, int N>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
operator*(const Eigen::Tensor<Scalar, 3>& lhs, const Eigen::Vector<Scalar, N>& rhs)
{
  const auto nx = lhs.dimension(0);
  const auto ny = lhs.dimension(1);
  const auto nz = lhs.dimension(2);

  assert(nz == rhs.size());

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(2, 0) };
  const Eigen::Tensor2Xd prod = lhs.contract(Eigen::TensorMap<const Eigen::Tensor1Xd>(rhs.data(), rhs.size()), dims);
  return Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(prod.data(), nx, ny);
}

template <typename Scalar, int N>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
operator*(const Eigen::RowVector<Scalar, N>& lhs, const Eigen::Tensor<Scalar, 3>& rhs)
{
  const auto nx = rhs.dimension(0);
  const auto ny = rhs.dimension(1);
  const auto nz = rhs.dimension(2);

  assert(lhs.size() == nx);

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(0, 0) };
  const Eigen::Tensor2Xd prod = Eigen::TensorMap<const Eigen::Tensor1Xd>(lhs.data(), lhs.size()).contract(rhs, dims);
  return Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(prod.data(), ny, nz);
}

template <typename Scalar, int Dim, int N, int M>
inline Eigen::Tensor<Scalar, Dim>
operator*(const Eigen::Tensor<Scalar, Dim>& lhs, const Eigen::Matrix<Scalar, N, M>& rhs)
{
  assert(lhs.dimension(Dim - 1) == rhs.rows());

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(Dim - 1, 0) };
  return lhs.contract(Eigen::TensorMap<const Eigen::Tensor2Xd>(rhs.data(), rhs.rows(), rhs.cols()), dims);
}

template <typename Scalar, int Dim, int N, int M>
inline Eigen::Tensor<Scalar, Dim>
operator*(const Eigen::Matrix<Scalar, N, M>& lhs, const Eigen::Tensor<Scalar, Dim>& rhs)
{
  assert(lhs.cols() == rhs.dimension(0));

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(1, 0) };
  return Eigen::TensorMap<const Eigen::Tensor2Xd>(lhs.data(), lhs.rows(), lhs.cols()).contract(rhs, dims);
}
