// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <unordered_set>

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/tree.hpp>

#include <tobas_kdl_msgs/msg/tree.hpp>

#include "./segment.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::kdl::Tree, tobas_kdl_msgs::msg::Tree>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::kdl::Tree;
  using ros_message_type = tobas_kdl_msgs::msg::Tree;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.segments.clear();

    for (const auto& [_, elem] : src.getSegments()) {
      dst.segments.emplace_back();
      tobas_kdl_msgs::SegmentAdapter::convert_to_ros_message(elem.segment, dst.segments.back().segment);
      dst.segments.back().q_nr = elem.q_nr;

      // Add the parent name unless this is the root link.
      if (elem.segment.name() != src.getRootName()) {
        dst.segments.back().parent_name = elem.parent->first;
      }
    }

    dst.root_name = src.getRootName();
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas::kdl::Tree tree(src.root_name);

    std::unordered_set<std::string> added_segs;  // Link names added to the tree
    added_segs.insert(src.root_name);            // The root link is included when the tree is created.

    tobas::kdl::Segment seg;
    size_t q_nr = 0;  // Current movable joint number

    for (size_t _ = 0; _ < src.segments.size(); ++_) {
      for (const auto& elem : src.segments) {
        // Skip links that have already been added.
        if (added_segs.contains(elem.segment.name)) {
          continue;
        }

        // To keep `q_nr` consistent, add movable joints to the tree in ascending order.
        // Links with fixed joints have number 0, so they are always candidates for addition.
        if (elem.q_nr > q_nr) {
          continue;
        }

        // Cannot add the link yet if its parent link has not been added.
        if (!added_segs.contains(elem.parent_name)) {
          continue;
        }

        // Add the current link to the tree.
        tobas_kdl_msgs::SegmentAdapter::convert_to_custom(elem.segment, seg);
        if (!tree.addSegment(seg, elem.parent_name)) {
          throw std::runtime_error("Failed to add segment \"" + elem.segment.name + "\".");
        }
        added_segs.insert(elem.segment.name);

        // For movable joints, search for the link with the next joint number.
        if (elem.segment.joint.type != tobas::kdl::Joint::kFixed) {
          ++q_nr;
        }
      }

      // Copy the tree and finish once all links have been added.
      if (added_segs.size() == src.segments.size()) {
        dst = tree;
        return;
      }
    }

    // At least one link should be added in each loop, so something is wrong
    // if the end condition is not met after looping over the number of links.
    throw std::runtime_error("Failed to convert tobas_kdl_msgs/Tree to tobas::kdl::Tree.");
  }
};

namespace tobas_kdl_msgs
{
using TreeAdapter = rclcpp::TypeAdapter<tobas::kdl::Tree, tobas_kdl_msgs::msg::Tree>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::kdl::Tree, tobas_kdl_msgs::msg::Tree);
