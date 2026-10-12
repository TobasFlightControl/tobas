// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cassert>
#include <expected>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "./segment.hpp"

namespace tobas
{
namespace kdl
{
class TreeElement;
using SegmentMap = std::map<std::string, TreeElement>;

class TreeElement
{
public:
  Segment segment;
  size_t q_nr;
  SegmentMap::const_iterator parent;
  std::vector<SegmentMap::const_iterator> children;

  inline explicit TreeElement(const Segment& _segment, const SegmentMap::const_iterator& _parent, size_t _q_nr);

  static inline TreeElement Root(const std::string& root_name);

private:
  inline explicit TreeElement(const std::string& name);
};

/** This class encapsulates a tree kinematic interconnection structure. It is built out of segments. */
class Tree
{
public:
  using SharedPtr = std::shared_ptr<Tree>;
  using ConstSharedPtr = std::shared_ptr<const Tree>;

  /** The constructor of a tree, a new tree is always empty. */
  explicit Tree(const std::string& root_name = "");

  /**
   * Copy constructor.
   * Since `TreeElement` member variables contain pointers,
   * an explicit copy constructor is required to copy objects.
   */
  Tree(const Tree& arg);
  Tree& operator=(const Tree& arg);

  /** Floating-link system with 6 DoF. */
  static Tree FloatingBase(const std::string& world_name, const std::string& base_name);

  /** Clear all segments. */
  void clear();

  /** Check validity. */
  std::expected<void, std::string> validate() const;

  /**
   * Adds a new segment to the end of the segment with hook_name as seg_name.
   *
   * @param segment new segment to add
   * @param hook_name name of the segment to connect this segment with
   */
  bool addSegment(const Segment& segment, const std::string& hook_name);

  /**
   * Removes a leaf segment without invalidating other segment iterators.
   *
   * @param seg_name name of the segment to remove from the tree
   */
  bool removeSegment(const std::string& seg_name);

  /**
   * Adds a complete tree to the end of the segment with hookname as seg_name.
   *
   * @param tree Tree to add
   * @param hook_name name of the segment to connect the tree with
   */
  bool addTree(const Tree& tree, const std::string& hook_name);

  /**
   * Extract a tree having seg_name as root. Only child segments of seg_name are added to the new tree.
   *
   * @param seg_name The name of the segment to be used as root of the new tree
   * @param tree The resulting sub-tree
   * @param root_mass_ok If false and the new root segment has mass, it will throw an exception.
   */
  bool getSubTree(const std::string& seg_name, Tree& tree, bool root_mass_ok = false) const;

  inline size_t getNrOfJoints() const;
  inline size_t getNrOfSegments() const;
  inline const SegmentMap& getSegments() const;
  inline SegmentMap::const_iterator getSegment(const std::string& seg_name) const;
  inline SegmentMap::const_iterator getRootSegment() const;
  inline const std::string& getRootName() const;

  inline bool empty() const;

  inline bool hasSegment(const std::string& seg_name) const;

  bool isEndSegment(const std::string& seg_name) const;
  bool isFixedToRoot(const std::string& seg_name) const;

  friend std::ostream& operator<<(std::ostream& os, const Tree& arg);

private:
  SegmentMap segments_;
  SegmentMap::const_iterator root_seg_ = segments_.end();
  size_t nj_ = 0;  ///< The number of movable joints

  std::expected<void, std::string> validateRecursive(
    const SegmentMap::const_iterator& seg_it,
    std::unordered_set<std::string>& seg_names,
    std::unordered_set<std::string>& jnt_names) const;
  bool addTreeRecursive(const SegmentMap::const_iterator& seg, const std::string& hook_name);
};

inline TreeElement::TreeElement(const Segment& _segment, const SegmentMap::const_iterator& _parent, size_t _q_nr)
  : segment(_segment), q_nr(_q_nr), parent(_parent)
{
}

inline TreeElement TreeElement::Root(const std::string& root_name)
{
  return TreeElement(root_name);
}

inline TreeElement::TreeElement(const std::string& name) : segment(name), q_nr(0)
{
}

inline size_t Tree::getNrOfJoints() const
{
  return nj_;
}

inline size_t Tree::getNrOfSegments() const
{
  return segments_.size();
}

inline const SegmentMap& Tree::getSegments() const
{
  return segments_;
}

inline SegmentMap::const_iterator Tree::getSegment(const std::string& seg_name) const
{
  return segments_.find(seg_name);
}

inline SegmentMap::const_iterator Tree::getRootSegment() const
{
  return root_seg_;
}

inline const std::string& Tree::getRootName() const
{
  assert(root_seg_ != segments_.end());
  return root_seg_->first;
}

inline bool Tree::empty() const
{
  return getNrOfSegments() == 0;
}

inline bool Tree::hasSegment(const std::string& seg_name) const
{
  return segments_.contains(seg_name);
}
}  // namespace kdl
}  // namespace tobas
