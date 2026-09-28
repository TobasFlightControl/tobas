// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl_parser/kdl_parser.hpp"

#include <tobas_kdl_conversions/kdl_urdf.hpp>

using namespace std;

namespace tobas
{
namespace kdl
{
TreeParser::TreeParser()
{
}

std::expected<void, std::string> TreeParser::parseFromPath(const string& path, Tree& tree)
{
  const auto model = urdf_parser_.parseFromPath(path);
  if (!model) {
    return std::unexpected(model.error());
  }

  return parseFromUrdf(**model, tree);
}

std::expected<void, std::string> TreeParser::parseFromText(const string& xml, Tree& tree)
{
  const auto model = urdf_parser_.parseFromText(xml);
  if (!model) {
    return std::unexpected(model.error());
  }

  return parseFromUrdf(**model, tree);
}

std::expected<void, std::string> TreeParser::parseFromUrdf(const ::urdf::ModelInterface& model, Tree& tree)
{
  const auto root_link = model.getRoot();
  if (!root_link) {
    return std::unexpected("Failed to get root link.");
  }

  tree = Tree(root_link->name);

  // Error if root link has inertia. KDL does not support this.
  if (root_link->inertial) {
    return std::unexpected(
      "The root link '" + root_link->name +
      "' has an inertia specified in the URDF, "
      "but KDL does not support a root link with an inertia. "
      "As a workaround, you can add an extra dummy link to your URDF.");
  }

  // Add all children.
  for (const auto& child : root_link->child_links) {
    if (const auto result = addChildrenToTree(child, tree); !result) {
      return result;
    }
  }

  return {};
}

std::expected<void, std::string> TreeParser::addChildrenToTree(const ::urdf::LinkConstSharedPtr& root, Tree& tree)
{
  // Construct the KDL joint.
  const auto joint = jointUrdfToKdl(*root->parent_joint);

  // Construct the tip frame.
  const auto f_tip = poseUrdfToKdl(root->parent_joint->parent_to_joint_origin_transform);

  // Construct the optional inertia.
  RigidBodyInertia inertia;
  if (root->inertial) {
    inertiaUrdfToKdl(*root->inertial, inertia);
  }

  // Construct the KDL segment.
  const Segment segment(root->name, joint, f_tip, inertia);

  // Add segment to tree.
  if (const auto result = tree.addSegment(segment, root->parent_joint->parent_link_name); !result) {
    return result;
  }

  // Recursively add all children.
  for (const auto& child : root->child_links) {
    if (const auto result = addChildrenToTree(child, tree); !result) {
      return result;
    }
  }

  return {};
}
}  // namespace kdl
}  // namespace tobas
