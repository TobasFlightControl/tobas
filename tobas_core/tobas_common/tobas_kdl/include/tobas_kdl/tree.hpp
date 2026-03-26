#pragma once

#include <memory>
#include <unordered_set>

#include "./chain.hpp"

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

/**
 * @brief This class encapsulates a tree kinematic interconnection structure.
 * It is built out of segments.
 */
class Tree
{
public:
  using SharedPtr = std::shared_ptr<Tree>;
  using ConstSharedPtr = std::shared_ptr<const Tree>;

  /**
   * @brief The constructor of a tree, a new tree is always empty
   */
  explicit Tree(const std::string& root_name = "");

  /**
   * @brief コピーコンストラクタ．
   * TreeElementのメンバ変数にポインタが含まれるため，オブジェクトをコピーするためには明示的にコピーコンストラクタを定義する必要がある．
   */
  Tree(const Tree& arg);
  Tree& operator=(const Tree& arg);

  /* 6DoFの浮遊リンク系． */
  static Tree FloatingBase(const std::string& world_name, const std::string& base_name);

  /* Clear all segments. */
  void clear();

  /* Check validity. */
  bool isValid(std::string& error_msg) const;

  /**
   * @brief Adds a new segment to the end of the segment with hook_name as seg_name.
   *
   * @param segment new segment to add
   * @param hook_name name of the segment to connect this segment with
   */
  bool addSegment(const Segment& segment, const std::string& hook_name);

  /**
   * @brief Adds a complete chain to the end of the segment with hook_name as seg_name.
   *
   * @param chain Chain to add
   * @param hook_name name of the segment to connect the chain with
   */
  bool addChain(const Chain& chain, const std::string& hook_name);

  /**
   * @brief Adds a complete tree to the end of the segment with hookname as seg_name.
   *
   * @param tree Tree to add
   * @param hook_name name of the segment to connect the tree with
   */
  bool addTree(const Tree& tree, const std::string& hook_name);

  /**
   * @brief Request the chain of the tree between root_name and tip_name.
   * The root_name and tip_name can be in different branches of the tree,
   * the root_name can be an ancestor of tip_name, and tip_name can be an ancestor of root_name.
   *
   * @param root_name the name of the root segment of the chain
   * @param tip_name the name of the tip segment of the chain
   * @param chain the resulting chain
   */
  bool getChain(const std::string& root_name, const std::string& tip_name, Chain& chain) const;

  /**
   * @brief Extract a tree having seg_name as root. Only child segments of seg_name are added to the new tree.
   *
   * @param seg_name The name of the segment to be used as root of the new tree
   * @param tree The resulting sub-tree
   * @param root_mass_ok If false and the new root segment has mass, it will throw an exception
   */
  bool getSubTree(const std::string& seg_name, Tree& tree, bool root_mass_ok = false) const;

  inline size_t getNrOfJoints() const;
  inline size_t getNrOfSegments() const;
  inline SegmentMap::const_iterator getSegment(const std::string& seg_name) const;
  inline SegmentMap::const_iterator getRootSegment() const;
  inline const std::string& getRootName() const;
  inline const SegmentMap& getSegments() const;

  inline bool empty() const;

  inline bool hasSegment(const std::string& seg_name) const;

  bool isEndSegment(const std::string& seg_name) const;
  bool isFixedToRoot(const std::string& seg_name) const;

  friend std::ostream& operator<<(std::ostream& os, const Tree& arg);

private:
  SegmentMap segments_;
  std::string root_name_;
  size_t nj_ = 0;
  size_t ns_ = 0;

  bool isValidRecursive(
    const SegmentMap::const_iterator& seg_it,
    std::unordered_set<std::string>& seg_names,
    std::unordered_set<std::string>& jnt_names,
    std::string& error_msg) const;
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

inline bool Tree::empty() const
{
  return ns_ == 0;
}

inline bool Tree::hasSegment(const std::string& seg_name) const
{
  return segments_.find(seg_name) != segments_.end();
}
}  // namespace kdl
}  // namespace tobas
