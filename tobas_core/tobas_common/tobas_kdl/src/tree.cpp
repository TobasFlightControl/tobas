// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/tree.cpp
// Modified by Tobas, Inc.

// Copyright  (C)  2007  Ruben Smits <ruben dot smits at intermodalics dot eu>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at intermodalics dot eu>
// Maintainer: Ruben Smits <ruben dot smits at intermodalics dot eu>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "tobas_kdl/tree.hpp"

#include <iostream>

namespace tobas::kdl
{
Tree::Tree(const std::string& _root_name)
{
  root_seg_ = segments_.insert(std::make_pair(_root_name, TreeElement::Root(_root_name))).first;
}

Tree::Tree(const Tree& _arg)
{
  *this = _arg;
}

Tree& Tree::operator=(const Tree& _arg)
{
  if (this == &_arg) {
    return *this;
  }

  clear();
  if (_arg.empty()) {
    return *this;
  }

  const auto& root_name = _arg.getRootName();
  root_seg_ = segments_.insert(std::make_pair(root_name, TreeElement::Root(root_name))).first;
  if (!addTree(_arg, root_name)) {
    throw std::runtime_error("Failed to add '" + root_name + "'.");
  }

  return *this;
}

Tree Tree::FloatingBase(const std::string& _world_name, const std::string& _base_name)
{
  static const std::string prefix = "floating_base_";
  static const std::string jnt_suffix = "_joint";

  Tree tree(_world_name);

  // X
  Joint x_jnt;
  const auto x_seg_name = prefix + "x";
  x_jnt.name = x_seg_name + jnt_suffix;
  x_jnt.type = Joint::kTranslation;
  x_jnt.axis(Vector::UnitX());
  const Segment x_seg(x_seg_name, x_jnt);
  if (!tree.addSegment(x_seg, _world_name)) {
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
  base_jnt.name = _base_name;
  base_jnt.type = Joint::kFixed;
  const Segment base_seg(_base_name, base_jnt);
  if (!tree.addSegment(base_seg, roll_seg_name)) {
    throw std::runtime_error("Failed to add '" + _base_name + "'");
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
  const SegmentMap::const_iterator& _seg_it,
  std::unordered_set<std::string>& _seg_names,
  std::unordered_set<std::string>& _jnt_names) const
{
  const auto& elem = _seg_it->second;
  const auto& seg = elem.segment;

  const auto& seg_name = seg.name();
  if (!_seg_names.insert(seg_name).second) {
    return std::unexpected("Segment name '" + seg_name + "' is duplicated.");
  }

  if (_seg_it != root_seg_) {
    const auto& jnt_name = seg.joint().name;
    if (!_jnt_names.insert(jnt_name).second) {
      return std::unexpected("Joint name '" + jnt_name + "' is duplicated.");
    }

    if (const auto result = seg.validate(); !result) {
      return result;
    }
  }

  for (const auto& child_it : elem.children) {
    if (const auto result = validateRecursive(child_it, _seg_names, _jnt_names); !result) {
      return result;
    }
  }

  return {};
}

bool Tree::addSegment(const Segment& _segment, const std::string& _hook_name)
{
  if (segments_.contains(_segment.name())) {
    std::cerr << "Segment '" + _segment.name() + "' already exists in the tree.\n";
    return false;
  }

  const auto parent = segments_.find(_hook_name);
  if (parent == segments_.end()) {
    std::cerr << "Segment '" + _hook_name + "' does not exist in the tree.\n";
    return false;
  }

  // Insert new element.
  const auto q_nr = _segment.joint().type != Joint::kFixed ? nj_ : 0;
  const auto retval = segments_.insert(std::make_pair(_segment.name(), TreeElement(_segment, parent, q_nr)));

  // Check if insertion succeeded.
  if (!retval.second) {
    std::cerr << "Failed to insert segment '" + _segment.name() + "' into the tree.\n";
    return false;
  }

  // Add iterator to new element in parents children list.
  parent->second.children.push_back(retval.first);

  // Increase number of joints.
  if (_segment.joint().type != Joint::kFixed) {
    ++nj_;
  }

  return true;
}

bool Tree::removeSegment(const std::string& _seg_name)
{
  const auto tar_it = getSegment(_seg_name);
  if (tar_it == segments_.end()) {
    std::cerr << "Segment '" << _seg_name << "' does not exist in the tree.\n";
    return false;
  }
  if (tar_it == root_seg_) {
    std::cerr << "Cannot remove root segment '" << _seg_name << "'.\n";
    return false;
  }

  const auto& tar_elem = tar_it->second;
  if (!tar_elem.children.empty()) {
    std::cerr << "Cannot remove segment '" << _seg_name << "' because it has children.\n";
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

bool Tree::addTree(const Tree& _tree, const std::string& _hook_name)
{
  return addTreeRecursive(_tree.getRootSegment(), _hook_name);
}

bool Tree::addTreeRecursive(const SegmentMap::const_iterator& _seg, const std::string& _hook_name)
{
  for (const auto& child : _seg->second.children) {
    if (!addSegment(child->second.segment, _hook_name)) {
      return false;
    }
    if (!addTreeRecursive(child, child->first)) {
      return false;
    }
  }

  return true;
}

bool Tree::getSubTree(const std::string& _seg_name, Tree& _tree, bool _root_mass_ok) const
{
  // Confirm that the specified segment exists.
  const auto seg_it = getSegment(_seg_name);
  if (seg_it == segments_.end()) {
    std::cerr << "Segment '" + _seg_name + "' does not exist in the tree.\n";
    return false;
  }

  // Confirm that the new root segment does not have mass.
  if (!_root_mass_ok) {
    const auto& segment = seg_it->second.segment;
    if (segment.inertia().getMass() > 0.0) {
      std::cerr << "KDL does not support a root segment with an inertia.\n";
      return false;
    }
  }

  // Initialize the tree.
  _tree = Tree(_seg_name);
  if (!_tree.addTreeRecursive(seg_it, _seg_name)) {
    return false;
  }

  return true;
}

bool Tree::isEndSegment(const std::string& _seg_name) const
{
  const auto seg_it = getSegment(_seg_name);
  assert(seg_it != segments_.end());
  return seg_it->second.children.empty();
}

bool Tree::isFixedToRoot(const std::string& _seg_name) const
{
  const auto seg_it = getSegment(_seg_name);
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

std::ostream& operator<<(std::ostream& _os, const Tree& _arg)
{
  for (const auto& [seg_name, elem] : _arg.segments_) {
    _os << "Segment:\n" << elem.segment << '\n';
    _os << "Number: " << elem.q_nr << '\n';

    _os << "Parent: ";
    if (seg_name != _arg.getRootName()) {
      _os << elem.parent->first << '\n';
    }
    else {
      _os << "-\n";
    }
  }

  return _os;
}
}  // namespace tobas::kdl
