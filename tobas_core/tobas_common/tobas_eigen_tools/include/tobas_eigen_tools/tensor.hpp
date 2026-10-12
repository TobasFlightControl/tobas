// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/unsupported/Eigen/CXX11/Tensor>

namespace Eigen
{
using Tensor1Xd = Tensor<double, 1>;
using Tensor2Xd = Tensor<double, 2>;
using Tensor3Xd = Tensor<double, 3>;
using Tensor4Xd = Tensor<double, 4>;
}  // namespace Eigen

namespace tobas
{
namespace eigen
{
template <typename TensorType>
inline TensorType shuffle(const TensorType& _tensor, const std::initializer_list<int>& _order)
{
  assert(_order.size() == TensorType::NumDimensions);

  Eigen::array<int, TensorType::NumDimensions> shuffle_order;
  std::copy(_order.begin(), _order.end(), shuffle_order.begin());
  return _tensor.shuffle(shuffle_order);
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
}  // namespace tobas

template <typename Scalar, int N>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
operator*(const Eigen::Tensor<Scalar, 3>& _lhs, const Eigen::Vector<Scalar, N>& _rhs)
{
  const auto nx = _lhs.dimension(0);
  const auto ny = _lhs.dimension(1);
  const auto nz = _lhs.dimension(2);

  assert(nz == _rhs.size());

  if (nz == 0) {
    return Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(nx, ny);
  }

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(2, 0) };
  const Eigen::TensorMap<const Eigen::Tensor1Xd> rhs_tensor(_rhs.data(), _rhs.size());
  const Eigen::Tensor2Xd prod = _lhs.contract(rhs_tensor, dims);
  return Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(prod.data(), nx, ny);
}

template <typename Scalar, int N>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
operator*(const Eigen::RowVector<Scalar, N>& _lhs, const Eigen::Tensor<Scalar, 3>& _rhs)
{
  const auto nx = _rhs.dimension(0);
  const auto ny = _rhs.dimension(1);
  const auto nz = _rhs.dimension(2);

  assert(_lhs.size() == nx);

  if (nx == 0) {
    return Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(ny, nz);
  }

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(0, 0) };
  const Eigen::TensorMap<const Eigen::Tensor1Xd> lhs_tensor(_lhs.data(), _lhs.size());
  const Eigen::Tensor2Xd prod = lhs_tensor.contract(_rhs, dims);
  return Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(prod.data(), ny, nz);
}

template <typename Scalar, int Dims, int N>
inline Eigen::Tensor<Scalar, Dims - 1>
operator*(const Eigen::Tensor<Scalar, Dims>& _lhs, const Eigen::Vector<Scalar, N>& _rhs)
{
  static_assert(Dims >= 4);
  assert(_lhs.dimension(Dims - 1) == _rhs.size());

  if (_rhs.size() == 0) {
    Eigen::DSizes<Eigen::DenseIndex, Dims - 1> dims;
    std::copy(_lhs.dimensions().begin(), _lhs.dimensions().end() - 1, dims.begin());
    Eigen::Tensor<Scalar, Dims - 1> res(dims);
    res.setZero();
    return res;
  }

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(Dims - 1, 0) };
  const Eigen::TensorMap<const Eigen::Tensor1Xd> rhs_tensor(_rhs.data(), _rhs.size());
  return _lhs.contract(rhs_tensor, dims);
}

template <typename Scalar, int Dims, int N>
inline Eigen::Tensor<Scalar, Dims - 1>
operator*(const Eigen::RowVector<Scalar, N>& _lhs, const Eigen::Tensor<Scalar, Dims>& _rhs)
{
  static_assert(Dims >= 4);
  assert(_lhs.size() == _rhs.dimension(Dims - 1));

  if (_lhs.size() == 0) {
    Eigen::DSizes<Eigen::DenseIndex, Dims - 1> dims;
    std::copy(_rhs.dimensions().begin() + 1, _rhs.dimensions().end(), dims.begin());
    Eigen::Tensor<Scalar, Dims - 1> res(dims);
    res.setZero();
    return res;
  }

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(0, 0) };
  const Eigen::TensorMap<const Eigen::Tensor1Xd> lhs_tensor(_lhs.data(), _lhs.size());
  return lhs_tensor.contract(_rhs, dims);
}

template <typename Scalar, int Dims, int N, int M>
inline Eigen::Tensor<Scalar, Dims>
operator*(const Eigen::Tensor<Scalar, Dims>& _lhs, const Eigen::Matrix<Scalar, N, M>& _rhs)
{
  static_assert(Dims >= 3);
  static_assert(M != 1);
  assert(_lhs.dimension(Dims - 1) == _rhs.rows());

  if (_rhs.rows() == 0) {
    Eigen::DSizes<Eigen::DenseIndex, Dims> dims;
    std::copy(_lhs.dimensions().begin(), _lhs.dimensions().end() - 1, dims.begin());
    dims.back() = _rhs.cols();
    Eigen::Tensor<Scalar, Dims> res(dims);
    res.setZero();
    return res;
  }

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(Dims - 1, 0) };
  const Eigen::TensorMap<const Eigen::Tensor2Xd> rhs_tensor(_rhs.data(), _rhs.rows(), _rhs.cols());
  return _lhs.contract(rhs_tensor, dims);
}

template <typename Scalar, int Dims, int N, int M>
inline Eigen::Tensor<Scalar, Dims>
operator*(const Eigen::Matrix<Scalar, N, M>& _lhs, const Eigen::Tensor<Scalar, Dims>& _rhs)
{
  static_assert(Dims >= 3);
  static_assert(N != 1);
  assert(_lhs.cols() == _rhs.dimension(0));

  if (_lhs.cols() == 0) {
    Eigen::DSizes<Eigen::DenseIndex, Dims> dims;
    dims.front() = _lhs.rows();
    std::copy(_rhs.dimensions().begin() + 1, _rhs.dimensions().end(), dims.begin() + 1);
    Eigen::Tensor<Scalar, Dims> res(dims);
    res.setZero();
    return res;
  }

  const Eigen::array<Eigen::IndexPair<int>, 1> dims = { Eigen::IndexPair<int>(1, 0) };
  const Eigen::TensorMap<const Eigen::Tensor2Xd> lhs_tensor(_lhs.data(), _lhs.rows(), _lhs.cols());
  return lhs_tensor.contract(_rhs, dims);
}
