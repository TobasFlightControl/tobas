// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/transforms.hpp"

#include <boost/algorithm/string/trim.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tobas_rviz_plugin/logger.hpp"

namespace tobas
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::getLogger("tobas.transforms");
}
}  // namespace

Transforms::Transforms(const std::string& target_frame) : target_frame_(target_frame)
{
  boost::trim(target_frame_);
  if (target_frame_.empty()) {
    RCLCPP_ERROR(getLogger(), "The target frame for Tobas Transforms cannot be empty.");
  }
  else {
    transforms_map_[target_frame_] = Eigen::Isometry3d::Identity();
  }
}

Transforms::~Transforms() = default;

const Eigen::Isometry3d& Transforms::getTransform(const std::string& from_frame) const
{
  if (!from_frame.empty()) {
    const auto it = transforms_map_.find(from_frame);
    if (it != transforms_map_.end()) {
      return it->second;
    }
  }

  RCLCPP_ERROR(
    getLogger(),
    "Unable to transform from frame '%s' to frame '%s'. Returning identity.",
    from_frame.c_str(),
    target_frame_.c_str());

  // If no transform found in map, return identity.
  static const auto identity = Eigen::Isometry3d::Identity();
  return identity;
}

bool Transforms::sameFrame(const std::string& frame1, const std::string& frame2)
{
  if (frame1.empty() || frame2.empty()) {
    return false;
  }
  return frame1 == frame2;
}
}  // namespace tobas
