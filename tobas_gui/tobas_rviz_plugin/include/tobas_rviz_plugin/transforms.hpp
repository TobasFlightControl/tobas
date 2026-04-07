// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>

#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace tobas
{
/**
 * @brief Provides an implementation of a snapshot of a transform tree
 * that can be easily queried for transforming different quantities.
 * Transforms are maintained as a list of transforms to a particular frame.
 * All stored transforms are considered fixed.
 */
class Transforms
{
  /* Map frame names to the transformation matrix that can transform objects from the frame name to the planning */
  using FixedTransformsMap = std::map<
    std::string,
    Eigen::Isometry3d,
    std::less<std::string>,
    Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>>;

public:
  /* Transforms cannot be copy-constructed */
  Transforms(const Transforms&) = delete;
  Transforms& operator=(const Transforms&) = delete;

  explicit Transforms(const std::string& target_frame);
  virtual ~Transforms();

  /* Check if two frames end up being the same once the missing / are added as prefix (if they are missing) */
  static bool sameFrame(const std::string& frame1, const std::string& frame2);

  /**
   * @brief Get transform for from_frame (w.r.t target frame)
   * @param from_frame The string id of the frame for which the transform is being computed
   * @return The required transform. It is guaranteed to be a valid isometry.
   */
  virtual const Eigen::Isometry3d& getTransform(const std::string& from_frame) const;

protected:
  std::string target_frame_;
  FixedTransformsMap transforms_map_;
};
}  // namespace tobas
