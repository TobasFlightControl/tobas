// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>
#include <vector>

#include <eigen3/Eigen/Core>

namespace tobas
{
namespace mission
{
/**
 * @brief Arc-length parameterized point on a Catmull-Rom path.
 *
 * `pos` is the position on the path. `tangent` is the unit direction with
 * respect to path length. `curvature` is d(tangent) / ds and can be used to
 * convert scalar path speed/acceleration into 3D velocity/acceleration:
 *   v = tangent * s_dot
 *   a = tangent * s_ddot + curvature * s_dot^2
 */
struct SplinePathPoint
{
  Eigen::Vector3d pos;
  Eigen::Vector3d tangent;
  Eigen::Vector3d curvature;
};

/**
 * @brief Catmull-Rom style 3D path through all given control points.
 *
 * The path uses cubic Hermite segments with Catmull-Rom tangents. End-point
 * tangents are one-sided so the first and last segments leave/enter the path
 * naturally. The class precomputes a sampled arc-length table, then accepts
 * distance along the path in get().
 *
 * This is intended for waypoint following: timing is handled separately by a
 * scalar trajectory generator, while this class only maps path distance to
 * position and geometric derivatives.
 */
class CatmullRomPath
{
  static constexpr size_t kSplineSamplesPerSegment = 50;

public:
  explicit CatmullRomPath(std::vector<Eigen::Vector3d> points);

  double length() const;
  SplinePathPoint get(double s) const;
  size_t segmentCount() const;

private:
  std::vector<Eigen::Vector3d> points_;
  std::vector<double> lengths_;

  Eigen::Vector3d tangentAt(size_t idx) const;
  Eigen::Vector3d position(size_t segment, double u) const;
  SplinePathPoint getBySegmentParameter(size_t segment, double u) const;
};
}  // namespace mission
}  // namespace tobas
