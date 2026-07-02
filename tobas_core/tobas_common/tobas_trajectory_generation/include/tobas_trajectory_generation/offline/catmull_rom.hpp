// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <ranges>
#include <utility>
#include <vector>

namespace tobas
{
namespace traj
{
/**
 * @brief Arc-length parameterized point on a Catmull-Rom path.
 *
 * `pos` is the position on the path. `tangent` is the unit direction with
 * respect to path length. `curvature` is d(tangent) / ds and can be used to
 * convert scalar path speed/acceleration into a vector trajectory:
 *   v = tangent * s_dot
 *   a = tangent * s_ddot + curvature * s_dot^2
 */
template <typename T>
struct CatmullRomPathPoint
{
  T pos;
  T tangent;
  T curvature;
};

/**
 * @brief Catmull-Rom style path through all given control points.
 *
 * The path uses cubic Hermite segments with Catmull-Rom tangents. End-point
 * tangents are one-sided so the first and last segments leave/enter the path
 * naturally. The class precomputes a sampled arc-length table, then accepts
 * distance along the path in get().
 */
template <typename T>
class CatmullRomPath
{
  static constexpr size_t kSplineSamplesPerSegment = 50;

public:
  explicit CatmullRomPath(std::vector<T> points) : points_(std::move(points))
  {
    assert(points_.size() >= 2);

    // get()で距離sから高速に区間を引けるよう，各セグメントを細かく刻んで弧長を近似する．
    lengths_.push_back(0.);
    for (size_t segment = 0; segment < segmentCount(); ++segment) {
      auto prev = position(segment, 0.);
      for (size_t sample = 1; sample <= kSplineSamplesPerSegment; ++sample) {
        const auto u = static_cast<double>(sample) / static_cast<double>(kSplineSamplesPerSegment);
        const auto cur = position(segment, u);
        lengths_.push_back(lengths_.back() + (cur - prev).norm());
        prev = cur;
      }
    }
  }

  double length() const
  {
    return lengths_.back();
  }

  CatmullRomPathPoint<T> get(double s) const
  {
    if (s <= 0.) {
      return getBySegmentParameter(0, 0.);
    }
    if (s >= length()) {
      return getBySegmentParameter(segmentCount() - 1, 1.);
    }

    // 弧長テーブル上でsを挟むサンプルを見つけ，その間を線形補間してセグメント内パラメータuへ戻す．
    const auto it = std::ranges::lower_bound(lengths_, s);
    const auto idx = std::distance(lengths_.begin(), it);
    const auto prev_idx = std::max<std::ptrdiff_t>(idx - 1, 0);
    const auto segment = static_cast<size_t>(prev_idx) / kSplineSamplesPerSegment;
    const auto sample = static_cast<size_t>(prev_idx) % kSplineSamplesPerSegment;

    const auto s0 = lengths_[prev_idx];
    const auto s1 = lengths_[idx];
    const auto ratio = s1 > s0 ? (s - s0) / (s1 - s0) : 0.;
    const auto u0 = static_cast<double>(sample) / static_cast<double>(kSplineSamplesPerSegment);
    const auto u1 = static_cast<double>(sample + 1) / static_cast<double>(kSplineSamplesPerSegment);
    return getBySegmentParameter(segment, u0 + (u1 - u0) * ratio);
  }

  size_t segmentCount() const
  {
    return points_.size() - 1;
  }

  T tangentAt(size_t idx) const
  {
    // 端点は隣接点との差分，内部点は前後点の中心差分を使うCatmull-Romの標準的な接線設定．
    if (idx == 0) {
      return points_[1] - points_[0];
    }
    if (idx == points_.size() - 1) {
      return points_[idx] - points_[idx - 1];
    }
    return 0.5 * (points_[idx + 1] - points_[idx - 1]);
  }

  T position(size_t segment, double u) const
  {
    // Catmull-Rom接線を持つ3次Hermite曲線として，指定セグメント内の位置を評価する．
    // cf. [Catmull–Rom spline](https://en.wikipedia.org/wiki/Catmull%E2%80%93Rom_spline)
    const auto& p0 = points_[segment];
    const auto& p1 = points_[segment + 1];
    const auto m0 = tangentAt(segment);
    const auto m1 = tangentAt(segment + 1);
    const auto u2 = u * u;
    const auto u3 = u2 * u;
    return (2 * u3 - 3 * u2 + 1) * p0 + (u3 - 2 * u2 + u) * m0 + (-2 * u3 + 3 * u2) * p1 + (u3 - u2) * m1;
  }

private:
  std::vector<T> points_;
  std::vector<double> lengths_;

  CatmullRomPathPoint<T> getBySegmentParameter(size_t segment, double u) const
  {
    const auto& p0 = points_[segment];
    const auto& p1 = points_[segment + 1];
    const auto m0 = tangentAt(segment);
    const auto m1 = tangentAt(segment + 1);
    const auto u2 = u * u;

    const auto pos = position(segment, u);
    const auto du = (6 * u2 - 6 * u) * p0 + (3 * u2 - 4 * u + 1) * m0 + (-6 * u2 + 6 * u) * p1 + (3 * u2 - 2 * u) * m1;
    const auto ddu = (12 * u - 6) * p0 + (6 * u - 4) * m0 + (-12 * u + 6) * p1 + (6 * u - 2) * m1;

    const auto du_norm = du.norm();
    if (du_norm == 0.) {
      // 重複点などで局所的に接線が定義できない場合は，速度・加速度指令を出さない．
      return { pos, T::Zero(), T::Zero() };
    }

    // u微分を弧長s微分へ変換し，s方向のスカラー軌道からベクトル加速度を作れる形にする．
    const auto tangent = du / du_norm;
    const auto curvature = (ddu * du_norm * du_norm - du * du.dot(ddu)) / std::pow(du_norm, 4);
    return { pos, tangent, curvature };
  }
};
}  // namespace traj
}  // namespace tobas
