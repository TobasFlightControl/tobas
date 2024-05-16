#include <sstream>

#include <tobas_std_tools/console.hpp>

#include "../include/tobas_kdl/tree.hpp"

using namespace std;

namespace tobas_kdl
{
Tree::Tree(const string& root_name) : nj_(0), ns_(0), root_name_(root_name)
{
  PRINT_DEBUG("Tree::Tree(\"" << root_name_ << "\")");

  segments_.insert(make_pair(root_name_, TreeElement::Root(root_name_)));
}

Tree::Tree(const Tree& in)
{
  PRINT_DEBUG("Tree::Tree(const Tree&)");

  segments_.clear();
  nj_ = 0;
  ns_ = 0;
  root_name_ = in.root_name_;

  segments_.insert(make_pair(root_name_, TreeElement::Root(root_name_)));
  addTree(in, root_name_);
}

Tree& Tree::operator=(const Tree& in)
{
  PRINT_DEBUG("Tree::operator=(const Tree&)");

  segments_.clear();
  nj_ = 0;
  ns_ = 0;
  root_name_ = in.root_name_;

  segments_.insert(make_pair(root_name_, TreeElement::Root(root_name_)));
  addTree(in, root_name_);

  return *this;
}

bool Tree::addSegment(const Segment& segment, const string& hook_name)
{
  const auto parent = segments_.find(hook_name);
  if (parent == segments_.end())
    return false;

  // Insert new element
  const auto q_nr = segment.getJoint().type != Joint::Fixed ? nj_ : 0;
  const auto retval =
    segments_.insert(make_pair(segment.name(), TreeElement(segment, parent, q_nr)));

  // check if insertion succeeded
  if (!retval.second)
    return false;

  // add iterator to new element in parents children list
  parent->second.children.push_back(retval.first);

  // increase number of segments
  ++ns_;

  // increase number of joints
  if (segment.getJoint().type != Joint::Fixed)
    ++nj_;

  return true;
}

bool Tree::addChain(const Chain& chain, const string& hook_name)
{
  string parent_name = hook_name;
  for (size_t i = 0; i < chain.getNrOfSegments(); ++i)
  {
    if (addSegment(chain.getSegment(i), parent_name))
      parent_name = chain.getSegment(i).name();
    else
      return false;
  }
  return true;
}

bool Tree::addTree(const Tree& tree, const string& hook_name)
{
  return addTreeRecursive(tree.getRootSegment(), hook_name);
}

bool Tree::addTreeRecursive(const SegmentMap::const_iterator& root, const string& hook_name)
{
  // get iterator for root-segment
  SegmentMap::const_iterator child;

  // try to add all of root's children
  for (size_t i = 0; i < root->second.children.size(); ++i)
  {
    child = root->second.children[i];
    // Try to add the child
    if (addSegment(child->second.segment, hook_name))
    {
      // If child is added, add all the child's children
      if (!(addTreeRecursive(child, child->first)))
        // If it didn't work, return false
        return false;
    }
    else
    {
      // If the child could not be added, return false
      return false;
    }
  }

  return true;
}

bool Tree::getChain(const string& chain_root, const string& chain_tip, Chain& chain) const
{
  // clear chain
  chain = Chain();

  // walk down from chain_root and chain_tip to the root of the tree
  vector<SegmentMap::key_type> parents_chain_root, parents_chain_tip;
  for (SegmentMap::const_iterator s = getSegment(chain_root); s != segments_.end();
       s = s->second.parent)
  {
    parents_chain_root.push_back(s->first);
    if (s->first == root_name_)
      break;
  }
  if (parents_chain_root.empty() || parents_chain_root.back() != root_name_)
    return false;
  for (SegmentMap::const_iterator s = getSegment(chain_tip); s != segments_.end();
       s = s->second.parent)
  {
    parents_chain_tip.push_back(s->first);
    if (s->first == root_name_)
      break;
  }
  if (parents_chain_tip.empty() || parents_chain_tip.back() != root_name_)
    return false;

  // remove common part of segment lists
  SegmentMap::key_type last_segment = root_name_;
  while (!parents_chain_root.empty() && !parents_chain_tip.empty()
         && parents_chain_root.back() == parents_chain_tip.back())
  {
    last_segment = parents_chain_root.back();
    parents_chain_root.pop_back();
    parents_chain_tip.pop_back();
  }
  parents_chain_root.push_back(last_segment);

  // add the segments from the root to the common frame
  for (size_t s = 0; s < parents_chain_root.size() - 1; ++s)
  {
    const Segment& seg = getSegment(parents_chain_root[s])->second.segment;
    const Frame& f_tip = seg.pose(0).inverse();
    Joint jnt = seg.getJoint();
    if (jnt.type == Joint::RotAxis)
    {
      jnt.type = Joint::RotAxis;
      jnt.origin = f_tip * jnt.origin;
      jnt.axis(f_tip.M * (-jnt.axis()));
    }
    else if (jnt.type == Joint::TransAxis)
    {
      jnt.type = Joint::TransAxis;
      jnt.origin = f_tip * jnt.origin;
      jnt.axis(f_tip.M * (-jnt.axis()));
    }
    chain.addSegment(Segment(
      getSegment(parents_chain_root[s + 1])->second.segment.name(), jnt, f_tip,
      getSegment(parents_chain_root[s + 1])->second.segment.getInertia()));
  }

  // add the segments from the common frame to the tip frame
  for (auto rit = parents_chain_tip.rbegin(); rit != parents_chain_tip.rend(); ++rit)
    chain.addSegment(getSegment(*rit)->second.segment);

  return true;
}

bool Tree::getSubTree(const string& segment_name, Tree& tree) const
{
  // check if segment_name exists
  const SegmentMap::const_iterator root = segments_.find(segment_name);
  if (root == segments_.end())
    return false;

  // init the tree, segment_name is the new root.
  tree = Tree(root->first);
  return tree.addTreeRecursive(root, segment_name);
}
}  // namespace tobas_kdl
