#pragma once

#include "./chain.hpp"

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

  inline explicit TreeElement(const Segment& _segment, const SegmentMap::const_iterator& _parent, const size_t& _q_nr);

  inline static TreeElement Root(const std::string& root_name);

private:
  inline explicit TreeElement(const std::string& name);
};

/**
 * \brief  This class encapsulates a tree kinematic interconnection structure.
 * It is built out of segments.
 */
class Tree
{
public:
  /**
   * The constructor of a tree, a new tree is always empty
   */
  explicit Tree(const std::string& root_name = "");

  /**
   * @brief コピーコンストラクタ．
   * TreeElementのメンバ変数にポインタが含まれるため，オブジェクトをコピーするためには明示的にコピーコンストラクタを定義する必要がある．
   */
  Tree(const Tree& in);
  Tree& operator=(const Tree& arg);

  /* 6DoFの浮遊リンク系． */
  static Tree FloatingBase(const std::string& world_name, const std::string& base_name);

  /**
   * Adds a new segment to the end of the segment with
   * hook_name as seg_name
   *
   * @param segment new segment to add
   * @param hook_name name of the segment to connect this
   * segment with.
   */
  void addSegment(const Segment& segment, const std::string& hook_name);

  /**
   * Adds a complete chain to the end of the segment with
   * hook_name as seg_name.
   *
   * @param hook_name name of the segment to connect the chain with.
   */
  void addChain(const Chain& chain, const std::string& hook_name);

  /**
   * Adds a complete tree to the end of the segment with
   * hookname as seg_name.
   *
   * @param tree Tree to add
   * @param hook_name name of the segment to connect the tree with
   */
  void addTree(const Tree& tree, const std::string& hook_name);

  /**
   * Request the chain of the tree between chain_root and chain_tip.  The chain_root
   * and chain_tip can be in different branches of the tree, the chain_root can be
   * an ancestor of chain_tip, and chain_tip can be an ancestor of chain_root.
   *
   * @param chain_root the name of the root segment of the chain
   * @param chain_tip the name of the tip segment of the chain
   * @param chain the resulting chain
   */
  void getChain(const std::string& chain_root, const std::string& chain_tip, Chain& chain) const;

  /**
   * Extract a tree having seg_name as root. Only child segments of
   * seg_name are added to the new tree.
   *
   * @param seg_name The name of the segment to be used as root of the new tree
   * @param tree The resulting sub-tree
   * @param root_mass_ok If false and the new root segment has mass, it will throw an exception
   */
  void getSubTree(const std::string& seg_name, Tree& tree, bool root_mass_ok = false) const;

  inline const size_t& getNrOfJoints() const;
  inline const size_t& getNrOfSegments() const;
  inline SegmentMap::const_iterator getSegment(const std::string& seg_name) const;
  inline SegmentMap::const_iterator getRootSegment() const;
  inline const std::string& getRootName() const;
  inline const SegmentMap& getSegments() const;

  inline bool hasSegment(const std::string& seg_name) const;

  friend std::ostream& operator<<(std::ostream& os, const Tree& arg);

private:
  SegmentMap segments_;
  std::string root_name_;
  size_t nj_ = 0;
  size_t ns_ = 0;

  void addTreeRecursive(const SegmentMap::const_iterator& seg, const std::string& hook_name);
};

inline TreeElement::TreeElement(const Segment& _segment, const SegmentMap::const_iterator& _parent, const size_t& _q_nr)
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

inline const size_t& Tree::getNrOfJoints() const
{
  return nj_;
}

inline const size_t& Tree::getNrOfSegments() const
{
  return ns_;
}

inline SegmentMap::const_iterator Tree::getSegment(const std::string& seg_name) const
{
  return segments_.find(seg_name);
}

inline SegmentMap::const_iterator Tree::getRootSegment() const
{
  return segments_.find(root_name_);
}

inline const std::string& Tree::getRootName() const
{
  return getRootSegment()->first;
}

inline const SegmentMap& Tree::getSegments() const
{
  return segments_;
}

inline bool Tree::hasSegment(const std::string& seg_name) const
{
  return segments_.find(seg_name) != segments_.end();
}
}  // namespace kdl
