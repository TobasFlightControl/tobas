#pragma once

#include "./chain.hpp"

namespace tobas_kdl
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

  inline explicit TreeElement(
    const Segment& _segment,
    const SegmentMap::const_iterator& _parent,
    const size_t& _q_nr)
    : segment(_segment), q_nr(_q_nr), parent(_parent)
  {
  }

  inline static TreeElement Root(const std::string& root_name)
  {
    return TreeElement(root_name);
  }

private:
  inline explicit TreeElement(const std::string& name) : segment(name), q_nr(0)
  {
  }
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
   * hook_name as segment_name
   *
   * @param segment new segment to add
   * @param hook_name name of the segment to connect this
   * segment with.
   */
  void addSegment(const Segment& segment, const std::string& hook_name);

  /**
   * Adds a complete chain to the end of the segment with
   * hook_name as segment_name.
   *
   * @param hook_name name of the segment to connect the chain with.
   */
  void addChain(const Chain& chain, const std::string& hook_name);

  /**
   * Adds a complete tree to the end of the segment with
   * hookname as segment_name.
   *
   * @param tree Tree to add
   * @param hook_name name of the segment to connect the tree with
   */
  void addTree(const Tree& tree, const std::string& hook_name);

  /**
   * Request the total number of joints in the tree.\n
   * Important: It is not the same as the total number of segments
   * since a segment does not need to have a joint.
   *
   * @return total nr of joints
   */
  inline const size_t& getNrOfJoints() const
  {
    return nj_;
  };

  /**
   * Request the total number of segments in the tree.
   * @return total number of segments
   */
  inline const size_t& getNrOfSegments() const
  {
    return ns_;
  };

  /**
   * Request the segment of the tree with name segment_name.
   *
   * @param segment_name the name of the requested segment
   *
   * @return constant iterator pointing to the requested segment
   */
  inline SegmentMap::const_iterator getSegment(const std::string& segment_name) const
  {
    return segments_.find(segment_name);
  };

  /**
   * Request the root segment of the tree
   *
   * @return constant iterator pointing to the root segment
   */
  inline SegmentMap::const_iterator getRootSegment() const
  {
    return segments_.find(root_name_);
  };

  /* Request the name of the root link. */
  inline const std::string& getRootName() const
  {
    return getRootSegment()->first;
  }

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
   * Extract a tree having segment_name as root. Only child segments of
   * segment_name are added to the new tree.
   *
   * @param segment_name the name of the segment to be used as root
   * of the new tree
   * @param tree the resulting sub-tree
   */
  void getSubTree(const std::string& segment_name, Tree& tree) const;

  inline const SegmentMap& getSegments() const
  {
    return segments_;
  }

  friend std::ostream& operator<<(std::ostream& os, const Tree& arg);

private:
  SegmentMap segments_;
  size_t nj_, ns_;
  std::string root_name_;

  void addTreeRecursive(const SegmentMap::const_iterator& seg, const std::string& hook_name);
};
}  // namespace tobas_kdl
