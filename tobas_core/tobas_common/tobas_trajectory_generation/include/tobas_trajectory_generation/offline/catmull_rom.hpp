// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <vector>

namespace tobas
{
namespace traj
{
/**
 * @brief Arc-length parameterized point on a Catmull-Rom path.
 *
 * `pos` is the position on the path. `tangent` is the unit direction with respect to path length.
 * `curvature` is d(tangent) / ds and can be used to convert scalar path speed/acceleration into a vector trajectory:
 *   v = tangent * s_dot
 *   a = tangent * s_ddot + curvature * s_dot^2
 */
template <typename Vector>
struct CatmullRomPathPoint
{
  Vector pos;
  Vector tangent;
  Vector curvature;
};

/**
 * @brief Catmull-Rom style path through all given control points.
 *
 * The path uses cubic Hermite segments with Catmull-Rom tangents.
 * End-point tangents are one-sided so the first and last segments leave/enter the path naturally.
 * The class precomputes a sampled arc-length table, then accepts distance along the path in get().
 */
template <typename Vector>
class CatmullRomPath
{
  static constexpr size_t kSplineSamplesPerSegment = 50;

public:
  /**
   * @brief Construct a path from ordered control points and precompute its arc-length table.
   *
   * The path contains one cubic segment between each adjacent pair of control points.
   * Throws std::invalid_argument when fewer than two control points are provided.
   */
  explicit CatmullRomPath(std::vector<Vector> points) : points_(std::move(points))
  {
    if (points_.size() < 2) {
      throw std::invalid_argument("CatmullRomPath requires at least two control points.");
    }

    // get()で距離sから高速に区間を引けるよう，各セグメントを細かく刻んで弧長を近似する．
    lengths_.push_back(0.);
    for (size_t segment = 0; segment < segmentCount(); ++segment) {
      auto prev = get(segment, 0.).pos;
      for (size_t sample = 1; sample <= kSplineSamplesPerSegment; ++sample) {
        const auto u = static_cast<double>(sample) / static_cast<double>(kSplineSamplesPerSegment);
        const auto cur = get(segment, u).pos;
        lengths_.push_back(lengths_.back() + (cur - prev).norm());
        prev = cur;
      }
    }
  }

  /**
   * @brief Return the number of cubic segments in the path.
   *
   * This is always one less than the number of control points.
   */
  size_t segmentCount() const
  {
    return points_.size() - 1;
  }

  /**
   * @brief Return the approximate total arc length of the path.
   *
   * The length is computed from the sampled arc-length table built at construction time.
   */
  double length() const
  {
    return lengths_.back();
  }

  /**
   * @brief Evaluate the path by arc length.
   *
   * @param s Distance from the start of the path. Values outside the path are clamped to the nearest endpoint.
   * @return Position, unit tangent, and curvature at the requested path distance.
   */
  CatmullRomPathPoint<Vector> get(double s) const
  {
    if (s <= 0.) {
      return get(0, 0.);
    }
    if (s >= length()) {
      return get(segmentCount() - 1, 1.);
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
    return get(segment, u0 + (u1 - u0) * ratio);
  }

  /**
   * @brief Evaluate the path by segment-local curve parameter.
   *
   * @param segment Segment index in [0, segmentCount()).
   * @param u Cubic segment parameter in [0, 1], not arc length.
   * @return Position, unit tangent, and curvature at the requested segment parameter.
   */
  CatmullRomPathPoint<Vector> get(size_t segment, double u) const
  {
    // Catmull-Rom接線を持つ3次Hermite曲線として，指定セグメント内の位置を評価する．
    // 任意の次元の任意の個数の順序を持つ点を滑らかにつなぐとき，ある2点間の曲線はその前後の2点を加えた4点のみで定まる．
    // cf. [Catmull–Rom spline](https://en.wikipedia.org/wiki/Catmull%E2%80%93Rom_spline)

    const auto& p0 = points_[segment];
    const auto& p1 = points_[segment + 1];
    const auto m0 = tangentAt(segment);
    const auto m1 = tangentAt(segment + 1);
    const auto u2 = u * u;
    const auto u3 = u2 * u;

    const auto pos = (2 * u3 - 3 * u2 + 1) * p0 + (u3 - 2 * u2 + u) * m0 + (-2 * u3 + 3 * u2) * p1 + (u3 - u2) * m1;
    const auto du = (6 * u2 - 6 * u) * p0 + (3 * u2 - 4 * u + 1) * m0 + (-6 * u2 + 6 * u) * p1 + (3 * u2 - 2 * u) * m1;
    const auto ddu = (12 * u - 6) * p0 + (6 * u - 4) * m0 + (-12 * u + 6) * p1 + (6 * u - 2) * m1;

    const auto du_norm = du.norm();
    if (du_norm == 0.) {
      // 重複点などで局所的に接線が定義できない場合は，速度・加速度指令を出さない．
      return { pos, Vector::Zero(), Vector::Zero() };
    }

    // u微分を弧長s微分へ変換し，s方向のスカラー軌道からベクトル加速度を作れる形にする．
    const auto tangent = du / du_norm;
    const auto curvature = (ddu * du_norm * du_norm - du * du.dot(ddu)) / std::pow(du_norm, 4);
    return { pos, tangent, curvature };
  }

private:
  std::vector<Vector> points_;
  std::vector<double> lengths_;

  /**
   * @brief Return the Catmull-Rom tangent assigned to a control point.
   *
   * Endpoint tangents use one-sided differences; interior tangents use the centered difference of neighboring points.
   */
  Vector tangentAt(size_t idx) const
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
};
}  // namespace traj
}  // namespace tobas
