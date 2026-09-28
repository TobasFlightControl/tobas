// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree.hpp"

#include <ostream>
#include <stdexcept>

using namespace std;

namespace tobas
{
namespace kdl
{
Tree::Tree(const string& root_name) : root_name_(root_name)
{
  segments_.insert(make_pair(root_name_, TreeElement::Root(root_name_)));
}

Tree::Tree(const Tree& arg)
{
  segments_.clear();
  root_name_ = arg.root_name_;
  nj_ = 0;
  ns_ = 0;

  segments_.insert(make_pair(root_name_, TreeElement::Root(root_name_)));
  if (const auto result = addTree(arg, root_name_); !result) {
    throw runtime_error("Failed to add '" + root_name_ + "': " + result.error());
  }
}

Tree& Tree::operator=(const Tree& arg)
{
  segments_.clear();
  root_name_ = arg.root_name_;
  nj_ = 0;
  ns_ = 0;

  segments_.insert(make_pair(root_name_, TreeElement::Root(root_name_)));
  if (const auto result = addTree(arg, root_name_); !result) {
    throw runtime_error("Failed to add '" + root_name_ + "': " + result.error());
  }

  return *this;
}

Tree Tree::FloatingBase(const string& world_name, const string& base_name)
{
  const string prefix = "floating_base_";
  const string jnt_suffix = "_joint";

  Tree tree(world_name);

  // X
  Joint x_jnt;
  const auto x_seg_name = prefix + "x";
  x_jnt.name = x_seg_name + jnt_suffix;
  x_jnt.type = Joint::kTranslation;
  x_jnt.axis(Vector::UnitX());
  const Segment x_seg(x_seg_name, x_jnt);
  if (const auto result = tree.addSegment(x_seg, world_name); !result) {
    throw runtime_error("Failed to add '" + x_seg_name + "': " + result.error());
  }

  // Y
  Joint y_jnt;
  const auto y_seg_name = prefix + "y";
  y_jnt.name = y_seg_name + jnt_suffix;
  y_jnt.type = Joint::kTranslation;
  y_jnt.axis(Vector::UnitY());
  const Segment y_seg(y_seg_name, y_jnt);
  if (const auto result = tree.addSegment(y_seg, x_seg_name); !result) {
    throw runtime_error("Failed to add '" + y_seg_name + "': " + result.error());
  }

  // Z
  Joint z_jnt;
  const auto z_seg_name = prefix + "z";
  z_jnt.name = z_seg_name + jnt_suffix;
  z_jnt.type = Joint::kTranslation;
  z_jnt.axis(Vector::UnitZ());
  const Segment z_seg(z_seg_name, z_jnt);
  if (const auto result = tree.addSegment(z_seg, y_seg_name); !result) {
    throw runtime_error("Failed to add '" + z_seg_name + "': " + result.error());
  }

  // Yaw
  Joint yaw_jnt;
  const auto yaw_seg_name = prefix + "yaw";
  yaw_jnt.name = yaw_seg_name + jnt_suffix;
  yaw_jnt.type = Joint::kRotation;
  yaw_jnt.axis(Vector::UnitZ());
  const Segment yaw_seg(yaw_seg_name, yaw_jnt);
  if (const auto result = tree.addSegment(yaw_seg, z_seg_name); !result) {
    throw runtime_error("Failed to add '" + yaw_seg_name + "': " + result.error());
  }

  // Pitch
  Joint pitch_jnt;
  const auto pitch_seg_name = prefix + "pitch";
  pitch_jnt.name = pitch_seg_name + jnt_suffix;
  pitch_jnt.type = Joint::kRotation;
  pitch_jnt.axis(Vector::UnitY());
  const Segment pitch_seg(pitch_seg_name, pitch_jnt);
  if (const auto result = tree.addSegment(pitch_seg, yaw_seg_name); !result) {
    throw runtime_error("Failed to add '" + pitch_seg_name + "': " + result.error());
  }

  // Roll
  Joint roll_jnt;
  const auto roll_seg_name = prefix + "roll";
  roll_jnt.name = roll_seg_name + jnt_suffix;
  roll_jnt.type = Joint::kRotation;
  roll_jnt.axis(Vector::UnitX());
  const Segment roll_seg(roll_seg_name, roll_jnt);
  if (const auto result = tree.addSegment(roll_seg, pitch_seg_name); !result) {
    throw runtime_error("Failed to add '" + roll_seg_name + "': " + result.error());
  }

  // Base
  Joint base_jnt;
  base_jnt.name = base_name;
  base_jnt.type = Joint::kFixed;
  const Segment base_seg(base_name, base_jnt);
  if (const auto result = tree.addSegment(base_seg, roll_seg_name); !result) {
    throw runtime_error("Failed to add '" + base_name + "': " + result.error());
  }

  return tree;
}

void Tree::clear()
{
  segments_.clear();
  root_name_.clear();
  nj_ = 0;
  ns_ = 0;
}

std::expected<void, std::string> Tree::isValid() const
{
  unordered_set<string> seg_names, jnt_names;
  return isValidRecursive(getRootSegment(), seg_names, jnt_names);
}

std::expected<void, std::string> Tree::isValidRecursive(
  const SegmentMap::const_iterator& seg_it,
  unordered_set<string>& seg_names,
  unordered_set<string>& jnt_names) const
{
  const auto& elem = seg_it->second;
  const auto& seg = elem.segment;

  const auto& seg_name = seg.name();
  if (!seg_names.insert(seg_name).second) {
    return std::unexpected("Segment name '" + seg_name + "' is duplicated.");
  }

  if (seg_it != getRootSegment()) {
    const auto& jnt_name = seg.joint().name;
    if (!jnt_names.insert(jnt_name).second) {
      return std::unexpected("Joint name '" + jnt_name + "' is duplicated.");
    }

    if (const auto result = seg.isValid(); !result) {
      return result;
    }
  }

  for (const auto& child_it : elem.children) {
    if (const auto result = isValidRecursive(child_it, seg_names, jnt_names); !result) {
      return result;
    }
  }

  return {};
}

std::expected<void, std::string> Tree::addSegment(const Segment& segment, const string& hook_name)
{
  if (segments_.contains(segment.name())) {
    return std::unexpected("Segment '" + segment.name() + "' already exists in the tree.");
  }

  const auto parent = segments_.find(hook_name);
  if (parent == segments_.end()) {
    return std::unexpected("Segment '" + hook_name + "' does not exist in the tree.");
  }

  // Insert new element.
  const auto q_nr = segment.joint().type != Joint::kFixed ? nj_ : 0;
  const auto retval = segments_.insert(make_pair(segment.name(), TreeElement(segment, parent, q_nr)));

  // Check if insertion succeeded.
  if (!retval.second) {
    return std::unexpected("Failed to insert segment '" + segment.name() + "' into the tree.");
  }

  // Add iterator to new element in parents children list.
  parent->second.children.push_back(retval.first);

  // Increase number of segments.
  ++ns_;

  // Increase number of joints.
  if (segment.joint().type != Joint::kFixed) {
    ++nj_;
  }

  return {};
}

std::expected<void, std::string> Tree::removeSegment(const string& seg_name)
{
  const auto seg_it = segments_.find(seg_name);
  if (seg_it == segments_.end()) {
    return std::unexpected("Segment '" + seg_name + "' does not exist in the tree.");
  }

  if (seg_it->first == root_name_) {
    return std::unexpected("Cannot remove root segment '" + seg_name + "'.");
  }
  if (!seg_it->second.children.empty()) {
    return std::unexpected("Cannot remove segment '" + seg_name + "' because it has children.");
  }

  // Keep movable joint indices contiguous and decrease the joint count.
  if (seg_it->second.segment.joint().type != Joint::kFixed) {
    const auto removed_index = seg_it->second.q_nr;
    for (auto& [_, element] : segments_) {
      if (element.segment.joint().type != Joint::kFixed && element.q_nr > removed_index) {
        --element.q_nr;
      }
    }
    --nj_;
  }

  // Remove the parent's reference before invalidating the segment iterator.
  const auto parent = segments_.find(seg_it->second.parent->first);
  std::erase(parent->second.children, seg_it);
  segments_.erase(seg_it);
  --ns_;

  return {};
}

std::expected<void, std::string> Tree::addChain(const Chain& chain, const string& hook_name)
{
  auto parent_name = hook_name;
  for (const auto& segment : chain.segments) {
    if (const auto result = addSegment(segment, parent_name); !result) {
      return result;
    }
    parent_name = segment.name();
  }

  return {};
}

std::expected<void, std::string> Tree::addTree(const Tree& tree, const string& hook_name)
{
  return addTreeRecursive(tree.getRootSegment(), hook_name);
}

std::expected<void, std::string> Tree::addTreeRecursive(const SegmentMap::const_iterator& seg, const string& hook_name)
{
  for (const auto& child : seg->second.children) {
    if (const auto result = addSegment(child->second.segment, hook_name); !result) {
      return result;
    }
    if (const auto result = addTreeRecursive(child, child->first); !result) {
      return result;
    }
  }

  return {};
}

std::expected<void, std::string> Tree::getChain(const string& root_name, const string& tip_name, Chain& chain) const
{
  // Clear chain.
  chain.clear();

  // Walk down from `root_name` and `tip_name` to the seg of the tree.
  vector<SegmentMap::key_type> parents_chain_root, parents_chain_tip;
  for (auto s = getSegment(root_name); s != segments_.end(); s = s->second.parent) {
    parents_chain_root.push_back(s->first);
    if (s->first == root_name_) {
      break;
    }
  }
  if (parents_chain_root.empty() || parents_chain_root.back() != root_name_) {
    return std::unexpected("Root segment '" + root_name + "' does not exist in the tree.");
  }

  for (auto s = getSegment(tip_name); s != segments_.end(); s = s->second.parent) {
    parents_chain_tip.push_back(s->first);
    if (s->first == root_name_) {
      break;
    }
  }
  if (parents_chain_tip.empty() || parents_chain_tip.back() != root_name_) {
    return std::unexpected("Tip segment '" + tip_name + "' does not exist in the tree.");
  }

  // Remove common part of segment lists.
  auto last_segment = root_name_;
  while (!parents_chain_root.empty() && !parents_chain_tip.empty() &&
         parents_chain_root.back() == parents_chain_tip.back()) {
    last_segment = parents_chain_root.back();
    parents_chain_root.pop_back();
    parents_chain_tip.pop_back();
  }
  parents_chain_root.push_back(last_segment);

  // Add the segments from the seg to the common frame.
  for (size_t s = 0; s < parents_chain_root.size() - 1; ++s) {
    const auto& seg = getSegment(parents_chain_root[s])->second.segment;
    const auto f_tip = seg.pose(0).inverse();
    auto jnt = seg.joint();
    if (jnt.type == Joint::kRotation) {
      jnt.type = Joint::kRotation;
      jnt.origin = f_tip * jnt.origin;
      jnt.axis(f_tip.M * (-jnt.axis()));
    }
    else if (jnt.type == Joint::kTranslation) {
      jnt.type = Joint::kTranslation;
      jnt.origin = f_tip * jnt.origin;
      jnt.axis(f_tip.M * (-jnt.axis()));
    }
    chain.addSegment(Segment(
      getSegment(parents_chain_root[s + 1])->second.segment.name(),
      jnt,
      f_tip,
      getSegment(parents_chain_root[s + 1])->second.segment.inertia()));
  }

  // Add the segments from the common frame to the tip frame.
  for (auto rit = parents_chain_tip.rbegin(); rit != parents_chain_tip.rend(); ++rit) {
    chain.addSegment(getSegment(*rit)->second.segment);
  }

  return {};
}

std::expected<void, std::string> Tree::getSubTree(const string& seg_name, Tree& tree, bool root_mass_ok) const
{
  // Confirm that the specified segment exists.
  const auto seg_it = segments_.find(seg_name);
  if (seg_it == segments_.end()) {
    return std::unexpected("Segment '" + seg_name + "' does not exist in the tree.");
  }

  // Confirm that the new root segment does not have mass.
  if (!root_mass_ok) {
    const auto& segment = seg_it->second.segment;
    if (segment.inertia().getMass() > 0) {
      return std::unexpected("KDL does not support a root segment with an inertia.");
    }
  }

  // Initialize the tree.
  tree = Tree(seg_name);
  return tree.addTreeRecursive(seg_it, seg_name);
}

bool Tree::isEndSegment(const string& seg_name) const
{
  const auto seg_it = segments_.find(seg_name);
  if (seg_it == segments_.end()) {
    return false;
  }
  return seg_it->second.children.empty();
}

bool Tree::isFixedToRoot(const string& seg_name) const
{
  if (seg_name == root_name_) {
    return true;
  }

  const auto seg_it = getSegment(seg_name);
  const auto& elem = seg_it->second;

  const auto& joint = elem.segment.joint();
  if (joint.type != Joint::kFixed) {
    return false;
  }

  const auto& parent_name = elem.parent->first;
  return isFixedToRoot(parent_name);
}

ostream& operator<<(ostream& os, const Tree& arg)
{
  for (const auto& [seg_name, elem] : arg.segments_) {
    os << "Segment:\n" << elem.segment << endl;
    os << "Number: " << elem.q_nr << endl;

    os << "Parent: ";
    if (seg_name != arg.root_name_) {
      os << elem.parent->first << endl;
    }
    else {
      os << "-" << endl;
    }
  }

  return os;
}
}  // namespace kdl
}  // namespace tobas
