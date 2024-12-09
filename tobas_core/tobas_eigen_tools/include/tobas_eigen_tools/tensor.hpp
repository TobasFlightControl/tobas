#pragma once

#include <eigen3/unsupported/Eigen/CXX11/Tensor>

namespace Eigen
{
using Tensor1Xd = Tensor<double, 1>;
using Tensor2Xd = Tensor<double, 2>;
using Tensor3Xd = Tensor<double, 3>;
using Tensor4Xd = Tensor<double, 4>;
}  // namespace Eigen

namespace eigen
{
template <typename TensorType>
inline TensorType shuffle(const TensorType& tensor, const std::initializer_list<int>& order)
{
  assert(order.size() == TensorType::NumDimensions);

  Eigen::array<int, TensorType::NumDimensions> shuffle_order;
  std::copy(order.begin(), order.end(), shuffle_order.begin());
  return tensor.shuffle(shuffle_order);
}

template <typename Scalar, int Dims, int N>
inline void setVectorX(
  Eigen::Tensor<Scalar, Dims>& _des,
  const Eigen::Vector<Scalar, N>& _src,
  const std::initializer_list<int>& _offset)
{
  assert(_offset.size() == Dims);

  Eigen::array<int, Dims> offset;
  std::copy(_offset.begin(), _offset.end(), offset.begin());

  Eigen::array<int, Dims> extent;
  extent.fill(1);
  extent.at(0) = _src.size();

  _des.slice(offset, extent) = Eigen::TensorMap<const Eigen::Tensor<Scalar, Dims>>(_src.data(), extent);
}
}  // namespace eigen

template <typename Scalar, int N>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
operator*(const Eigen::Tensor<Scalar, 3>& lhs, const Eigen::Vector<Scalar, N>& rhs)
{
  const auto nx = lhs.dimension(0);
  const auto ny = lhs.dimension(1);
  const auto nz = lhs.dimension(2);

  assert(nz == rhs.size());

  if (nz == 0)
    return Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(nx, ny);

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(2, 0) };
  const Eigen::TensorMap<const Eigen::Tensor1Xd> rhs_tensor(rhs.data(), rhs.size());
  const Eigen::Tensor2Xd prod = lhs.contract(rhs_tensor, dims);
  return Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(prod.data(), nx, ny);
}

template <typename Scalar, int N>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
operator*(const Eigen::RowVector<Scalar, N>& lhs, const Eigen::Tensor<Scalar, 3>& rhs)
{
  const auto nx = rhs.dimension(0);
  const auto ny = rhs.dimension(1);
  const auto nz = rhs.dimension(2);

  assert(lhs.size() == nx);

  if (nx == 0)
    return Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(ny, nz);

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(0, 0) };
  const Eigen::TensorMap<const Eigen::Tensor1Xd> lhs_tensor(lhs.data(), lhs.size());
  const Eigen::Tensor2Xd prod = lhs_tensor.contract(rhs, dims);
  return Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(prod.data(), ny, nz);
}

template <typename Scalar, int Dims, int N>
inline Eigen::Tensor<Scalar, Dims - 1>
operator*(const Eigen::Tensor<Scalar, Dims>& lhs, const Eigen::Vector<Scalar, N>& rhs)
{
  static_assert(Dims >= 4);
  assert(lhs.dimension(Dims - 1) == rhs.size());

  if (rhs.size() == 0)
  {
    Eigen::DSizes<Eigen::DenseIndex, Dims - 1> dims;
    std::copy(lhs.dimensions().begin(), lhs.dimensions().end() - 1, dims.begin());
    Eigen::Tensor<Scalar, Dims - 1> res(dims);
    res.setZero();
    return res;
  }

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(Dims - 1, 0) };
  const Eigen::TensorMap<const Eigen::Tensor1Xd> rhs_tensor(rhs.data(), rhs.size());
  return lhs.contract(rhs_tensor, dims);
}

template <typename Scalar, int Dims, int N>
inline Eigen::Tensor<Scalar, Dims - 1>
operator*(const Eigen::RowVector<Scalar, N>& lhs, const Eigen::Tensor<Scalar, Dims>& rhs)
{
  static_assert(Dims >= 4);
  assert(lhs.size() == rhs.dimension(Dims - 1));

  if (lhs.size() == 0)
  {
    Eigen::DSizes<Eigen::DenseIndex, Dims - 1> dims;
    std::copy(rhs.dimensions().begin() + 1, rhs.dimensions().end(), dims.begin());
    Eigen::Tensor<Scalar, Dims - 1> res(dims);
    res.setZero();
    return res;
  }

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(0, 0) };
  const Eigen::TensorMap<const Eigen::Tensor1Xd> lhs_tensor(lhs.data(), lhs.size());
  return lhs_tensor.contract(rhs, dims);
}

template <typename Scalar, int Dims, int N, int M>
inline Eigen::Tensor<Scalar, Dims>
operator*(const Eigen::Tensor<Scalar, Dims>& lhs, const Eigen::Matrix<Scalar, N, M>& rhs)
{
  static_assert(Dims >= 3);
  static_assert(M != 1);
  assert(lhs.dimension(Dims - 1) == rhs.rows());

  if (rhs.rows() == 0)
  {
    Eigen::DSizes<Eigen::DenseIndex, Dims> dims;
    std::copy(lhs.dimensions().begin(), lhs.dimensions().end() - 1, dims.begin());
    dims.back() = rhs.cols();
    Eigen::Tensor<Scalar, Dims> res(dims);
    res.setZero();
    return res;
  }

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(Dims - 1, 0) };
  const Eigen::TensorMap<const Eigen::Tensor2Xd> rhs_tensor(rhs.data(), rhs.rows(), rhs.cols());
  return lhs.contract(rhs_tensor, dims);
}

template <typename Scalar, int Dims, int N, int M>
inline Eigen::Tensor<Scalar, Dims>
operator*(const Eigen::Matrix<Scalar, N, M>& lhs, const Eigen::Tensor<Scalar, Dims>& rhs)
{
  static_assert(Dims >= 3);
  static_assert(N != 1);
  assert(lhs.cols() == rhs.dimension(0));

  if (lhs.cols() == 0)
  {
    Eigen::DSizes<Eigen::DenseIndex, Dims> dims;
    dims.front() = lhs.rows();
    std::copy(rhs.dimensions().begin() + 1, rhs.dimensions().end(), dims.begin() + 1);
    Eigen::Tensor<Scalar, Dims> res(dims);
    res.setZero();
    return res;
  }

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(1, 0) };
  const Eigen::TensorMap<const Eigen::Tensor2Xd> lhs_tensor(lhs.data(), lhs.rows(), lhs.cols());
  return lhs_tensor.contract(rhs, dims);
}
