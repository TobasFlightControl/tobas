// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

namespace tobas
{
namespace traj
{
/**
 * Arc-length parameterized point on a Catmull-Rom path.
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
 * Catmull-Rom style path through all given control points.
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
   * Construct a path from ordered control points and precompute its arc-length table.
   *
   * The path contains one cubic segment between each adjacent pair of control points.
   * Throws std::invalid_argument when fewer than two control points are provided.
   */
  explicit CatmullRomPath(std::vector<Vector> _points) : points_(std::move(_points))
  {
    if (points_.size() < 2) {
      throw std::invalid_argument("CatmullRomPath requires at least two control points.");
    }

    // Approximate the arc length by finely sampling each segment so `get()` can quickly find the segment from distance `s`.
    lengths_.push_back(0.0);
    for (size_t segment = 0; segment < segmentCount(); ++segment) {
      auto prev = get(segment, 0.0).pos;
      for (size_t sample = 1; sample <= kSplineSamplesPerSegment; ++sample) {
        const auto u = static_cast<double>(sample) / static_cast<double>(kSplineSamplesPerSegment);
        const auto cur = get(segment, u).pos;
        lengths_.push_back(lengths_.back() + (cur - prev).norm());
        prev = cur;
      }
    }
  }

  /**
   * Return the number of cubic segments in the path.
   *
   * This is always one less than the number of control points.
   */
  size_t segmentCount() const
  {
    return points_.size() - 1;
  }

  /**
   * Return the approximate total arc length of the path.
   *
   * The length is computed from the sampled arc-length table built at construction time.
   */
  double length() const
  {
    return lengths_.back();
  }

  /**
   * Evaluate the path by arc length.
   *
   * @param _s Distance from the start of the path. Values outside the path are clamped to the nearest endpoint.
   * @return Position, unit tangent, and curvature at the requested path distance.
   */
  CatmullRomPathPoint<Vector> get(double _s) const
  {
    if (_s <= 0.0) {
      return get(0, 0.0);
    }
    if (_s >= length()) {
      return get(segmentCount() - 1, 1.0);
    }

    // Find samples that bracket `s` in the arc-length table,
    // then linearly interpolate between them to recover the segment-local parameter `u`.
    const auto it = std::ranges::lower_bound(lengths_, _s);
    const auto idx = std::distance(lengths_.begin(), it);
    const auto prev_idx = std::max<std::ptrdiff_t>(idx - 1, 0);
    const auto segment = static_cast<size_t>(prev_idx) / kSplineSamplesPerSegment;
    const auto sample = static_cast<size_t>(prev_idx) % kSplineSamplesPerSegment;

    const auto s0 = lengths_[prev_idx];
    const auto s1 = lengths_[idx];
    const auto ratio = s1 > s0 ? (_s - s0) / (s1 - s0) : 0.0;
    const auto u0 = static_cast<double>(sample) / static_cast<double>(kSplineSamplesPerSegment);
    const auto u1 = static_cast<double>(sample + 1) / static_cast<double>(kSplineSamplesPerSegment);
    return get(segment, u0 + (u1 - u0) * ratio);
  }

  /**
   * Evaluate the path by segment-local curve parameter.
   *
   * @param _segment Segment index in [0, segmentCount()).
   * @param _u Cubic segment parameter in [0, 1], not arc length.
   * @return Position, unit tangent, and curvature at the requested segment parameter.
   */
  CatmullRomPathPoint<Vector> get(size_t _segment, double _u) const
  {
    // Evaluate the position in the specified segment as a cubic Hermite curve with Catmull-Rom tangents.
    // When smoothly connecting any number of ordered points in any dimension,
    // the curve between two points is determined
    // only by those two points plus the neighboring points before and after them.
    // Ref: [Catmull–Rom spline](https://en.wikipedia.org/wiki/Catmull%E2%80%93Rom_spline)

    assert(_segment + 1 < points_.size());

    const auto& p0 = points_[_segment];
    const auto& p1 = points_[_segment + 1];
    const auto m0 = tangentAt(_segment);
    const auto m1 = tangentAt(_segment + 1);
    const auto u2 = _u * _u;
    const auto u3 = u2 * _u;

    const auto pos = (2 * u3 - 3 * u2 + 1) * p0 + (u3 - 2 * u2 + _u) * m0 + (-2 * u3 + 3 * u2) * p1 + (u3 - u2) * m1;
    const auto du =
      (6 * u2 - 6 * _u) * p0 + (3 * u2 - 4 * _u + 1) * m0 + (-6 * u2 + 6 * _u) * p1 + (3 * u2 - 2 * _u) * m1;
    const auto ddu = (12 * _u - 6) * p0 + (6 * _u - 4) * m0 + (-12 * _u + 6) * p1 + (6 * _u - 2) * m1;

    const auto du_norm = du.norm();
    if (du_norm == 0.0) {
      // If the tangent cannot be defined locally, for example due to duplicate points,
      // do not issue velocity or acceleration commands.
      return { pos, Vector::Zero(), Vector::Zero() };
    }

    // Convert u derivatives to arc-length s derivatives
    // so vector acceleration can be built from a scalar trajectory in the s direction.
    const auto tangent = du / du_norm;
    const auto curvature = (ddu * du_norm * du_norm - du * du.dot(ddu)) / std::pow(du_norm, 4);
    return { pos, tangent, curvature };
  }

private:
  std::vector<Vector> points_;
  std::vector<double> lengths_;

  /**
   * Return the Catmull-Rom tangent assigned to a control point.
   *
   * Endpoint tangents use one-sided differences; interior tangents use the centered difference of neighboring points.
   */
  Vector tangentAt(size_t _idx) const
  {
    // Standard Catmull-Rom tangent setting: endpoints use one-sided differences to neighboring points,
    // and interior points use centered differences between the previous and next points.
    if (_idx == 0) {
      return points_[1] - points_[0];
    }
    if (_idx == points_.size() - 1) {
      return points_[_idx] - points_[_idx - 1];
    }
    return 0.5 * (points_[_idx + 1] - points_[_idx - 1]);
  }
};
}  // namespace traj
}  // namespace tobas
