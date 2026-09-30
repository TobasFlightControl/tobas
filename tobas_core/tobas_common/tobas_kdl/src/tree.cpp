// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree.hpp"

#include <iostream>

namespace tobas
{
namespace kdl
{
Tree::Tree(const std::string& root_name)
{
  root_seg_ = segments_.insert(std::make_pair(root_name, TreeElement::Root(root_name))).first;
}

Tree::Tree(const Tree& arg)
{
  *this = arg;
}

Tree& Tree::operator=(const Tree& arg)
{
  if (this == &arg) {
    return *this;
  }

  clear();
  if (arg.empty()) {
    return *this;
  }

  const auto& root_name = arg.getRootName();
  root_seg_ = segments_.insert(std::make_pair(root_name, TreeElement::Root(root_name))).first;
  if (!addTree(arg, root_name)) {
    throw std::runtime_error("Failed to add '" + root_name + "'.");
  }

  return *this;
}

Tree Tree::FloatingBase(const std::string& world_name, const std::string& base_name)
{
  const std::string prefix = "floating_base_";
  const std::string jnt_suffix = "_joint";

  Tree tree(world_name);

  // X
  Joint x_jnt;
  const auto x_seg_name = prefix + "x";
  x_jnt.name = x_seg_name + jnt_suffix;
  x_jnt.type = Joint::kTranslation;
  x_jnt.axis(Vector::UnitX());
  const Segment x_seg(x_seg_name, x_jnt);
  if (!tree.addSegment(x_seg, world_name)) {
    throw std::runtime_error("Failed to add '" + x_seg_name + "'");
  }

  // Y
  Joint y_jnt;
  const auto y_seg_name = prefix + "y";
  y_jnt.name = y_seg_name + jnt_suffix;
  y_jnt.type = Joint::kTranslation;
  y_jnt.axis(Vector::UnitY());
  const Segment y_seg(y_seg_name, y_jnt);
  if (!tree.addSegment(y_seg, x_seg_name)) {
    throw std::runtime_error("Failed to add '" + y_seg_name + "'");
  }

  // Z
  Joint z_jnt;
  const auto z_seg_name = prefix + "z";
  z_jnt.name = z_seg_name + jnt_suffix;
  z_jnt.type = Joint::kTranslation;
  z_jnt.axis(Vector::UnitZ());
  const Segment z_seg(z_seg_name, z_jnt);
  if (!tree.addSegment(z_seg, y_seg_name)) {
    throw std::runtime_error("Failed to add '" + z_seg_name + "'");
  }

  // Yaw
  Joint yaw_jnt;
  const auto yaw_seg_name = prefix + "yaw";
  yaw_jnt.name = yaw_seg_name + jnt_suffix;
  yaw_jnt.type = Joint::kRotation;
  yaw_jnt.axis(Vector::UnitZ());
  const Segment yaw_seg(yaw_seg_name, yaw_jnt);
  if (!tree.addSegment(yaw_seg, z_seg_name)) {
    throw std::runtime_error("Failed to add '" + yaw_seg_name + "'");
  }

  // Pitch
  Joint pitch_jnt;
  const auto pitch_seg_name = prefix + "pitch";
  pitch_jnt.name = pitch_seg_name + jnt_suffix;
  pitch_jnt.type = Joint::kRotation;
  pitch_jnt.axis(Vector::UnitY());
  const Segment pitch_seg(pitch_seg_name, pitch_jnt);
  if (!tree.addSegment(pitch_seg, yaw_seg_name)) {
    throw std::runtime_error("Failed to add '" + pitch_seg_name + "'");
  }

  // Roll
  Joint roll_jnt;
  const auto roll_seg_name = prefix + "roll";
  roll_jnt.name = roll_seg_name + jnt_suffix;
  roll_jnt.type = Joint::kRotation;
  roll_jnt.axis(Vector::UnitX());
  const Segment roll_seg(roll_seg_name, roll_jnt);
  if (!tree.addSegment(roll_seg, pitch_seg_name)) {
    throw std::runtime_error("Failed to add '" + roll_seg_name + "'");
  }

  // Base
  Joint base_jnt;
  base_jnt.name = base_name;
  base_jnt.type = Joint::kFixed;
  const Segment base_seg(base_name, base_jnt);
  if (!tree.addSegment(base_seg, roll_seg_name)) {
    throw std::runtime_error("Failed to add '" + base_name + "'");
  }

  return tree;
}

void Tree::clear()
{
  segments_.clear();
  root_seg_ = segments_.end();
  nj_ = 0;
}

std::expected<void, std::string> Tree::validate() const
{
  std::unordered_set<std::string> seg_names, jnt_names;
  return validateRecursive(root_seg_, seg_names, jnt_names);
}

std::expected<void, std::string> Tree::validateRecursive(
  const SegmentMap::const_iterator& seg_it,
  std::unordered_set<std::string>& seg_names,
  std::unordered_set<std::string>& jnt_names) const
{
  const auto& elem = seg_it->second;
  const auto& seg = elem.segment;

  const auto& seg_name = seg.name();
  if (!seg_names.insert(seg_name).second) {
    return std::unexpected("Segment name '" + seg_name + "' is duplicated.");
  }

  if (seg_it != root_seg_) {
    const auto& jnt_name = seg.joint().name;
    if (!jnt_names.insert(jnt_name).second) {
      return std::unexpected("Joint name '" + jnt_name + "' is duplicated.");
    }

    if (const auto result = seg.validate(); !result) {
      return result;
    }
  }

  for (const auto& child_it : elem.children) {
    if (const auto result = validateRecursive(child_it, seg_names, jnt_names); !result) {
      return result;
    }
  }

  return {};
}

bool Tree::addSegment(const Segment& segment, const std::string& hook_name)
{
  if (segments_.contains(segment.name())) {
    std::cerr << "Segment '" + segment.name() + "' already exists in the tree." << std::endl;
    return false;
  }

  const auto parent = segments_.find(hook_name);
  if (parent == segments_.end()) {
    std::cerr << "Segment '" + hook_name + "' does not exist in the tree." << std::endl;
    return false;
  }

  // Insert new element.
  const auto q_nr = segment.joint().type != Joint::kFixed ? nj_ : 0;
  const auto retval = segments_.insert(std::make_pair(segment.name(), TreeElement(segment, parent, q_nr)));

  // Check if insertion succeeded.
  if (!retval.second) {
    std::cerr << "Failed to insert segment '" + segment.name() + "' into the tree." << std::endl;
    return false;
  }

  // Add iterator to new element in parents children list.
  parent->second.children.push_back(retval.first);

  // Increase number of joints.
  if (segment.joint().type != Joint::kFixed) {
    ++nj_;
  }

  return true;
}

bool Tree::removeSegment(const std::string& seg_name)
{
  const auto tar_it = getSegment(seg_name);
  if (tar_it == segments_.end()) {
    std::cerr << "Segment '" << seg_name << "' does not exist in the tree." << std::endl;
    return false;
  }
  if (tar_it == root_seg_) {
    std::cerr << "Cannot remove root segment '" << seg_name << "'." << std::endl;
    return false;
  }

  const auto& tar_elem = tar_it->second;
  if (!tar_elem.children.empty()) {
    std::cerr << "Cannot remove segment '" << seg_name << "' because it has children." << std::endl;
    return false;
  }

  // Keep movable joint indices contiguous and decrease the joint count.
  if (tar_elem.segment.joint().type != Joint::kFixed) {
    for (auto& [_, elem] : segments_) {
      if (elem.segment.joint().type != Joint::kFixed && elem.q_nr > tar_elem.q_nr) {
        --elem.q_nr;
      }
    }
    --nj_;
  }

  // Remove the parent's reference before invalidating the segment iterator.
  const auto par_it = segments_.find(tar_elem.parent->first);
  std::erase(par_it->second.children, tar_it);
  segments_.erase(tar_it);

  return true;
}

bool Tree::addTree(const Tree& tree, const std::string& hook_name)
{
  return addTreeRecursive(tree.getRootSegment(), hook_name);
}

bool Tree::addTreeRecursive(const SegmentMap::const_iterator& seg, const std::string& hook_name)
{
  for (const auto& child : seg->second.children) {
    if (!addSegment(child->second.segment, hook_name)) {
      return false;
    }
    if (!addTreeRecursive(child, child->first)) {
      return false;
    }
  }

  return true;
}

bool Tree::getSubTree(const std::string& seg_name, Tree& tree, bool root_mass_ok) const
{
  // Confirm that the specified segment exists.
  const auto seg_it = getSegment(seg_name);
  if (seg_it == segments_.end()) {
    std::cerr << "Segment '" + seg_name + "' does not exist in the tree." << std::endl;
    return false;
  }

  // Confirm that the new root segment does not have mass.
  if (!root_mass_ok) {
    const auto& segment = seg_it->second.segment;
    if (segment.inertia().getMass() > 0.0) {
      std::cerr << "KDL does not support a root segment with an inertia." << std::endl;
      return false;
    }
  }

  // Initialize the tree.
  tree = Tree(seg_name);
  if (!tree.addTreeRecursive(seg_it, seg_name)) {
    return false;
  }

  return true;
}

bool Tree::isEndSegment(const std::string& seg_name) const
{
  const auto seg_it = getSegment(seg_name);
  assert(seg_it != segments_.end());
  return seg_it->second.children.empty();
}

bool Tree::isFixedToRoot(const std::string& seg_name) const
{
  const auto seg_it = getSegment(seg_name);
  assert(seg_it != segments_.end());

  if (seg_it == root_seg_) {
    return true;
  }

  const auto& elem = seg_it->second;

  const auto& joint = elem.segment.joint();
  if (joint.type != Joint::kFixed) {
    return false;
  }

  const auto& parent_name = elem.parent->first;
  return isFixedToRoot(parent_name);
}

std::ostream& operator<<(std::ostream& os, const Tree& arg)
{
  for (const auto& [seg_name, elem] : arg.segments_) {
    os << "Segment:\n" << elem.segment << std::endl;
    os << "Number: " << elem.q_nr << std::endl;

    os << "Parent: ";
    if (seg_name != arg.getRootName()) {
      os << elem.parent->first << std::endl;
    }
    else {
      os << "-" << std::endl;
    }
  }

  return os;
}
}  // namespace kdl
}  // namespace tobas
