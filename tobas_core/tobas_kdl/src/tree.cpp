#include <tobas_std_tools/map.hpp>
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

Tree Tree::FloatingBase(const string& world_name, const string& base_name)
{
  const string prefix = "floating_base_";
  const string jnt_suffix = "_joint";

  Tree tree(world_name);

  // X
  Joint x_jnt;
  const auto x_seg_name = prefix + "x";
  x_jnt.name = x_seg_name + jnt_suffix;
  x_jnt.type = Joint::TransAxis;
  x_jnt.axis(Vector::UnitX());
  Segment x_seg(x_seg_name, x_jnt);
  tree.addSegment(x_seg, world_name);

  // Y
  Joint y_jnt;
  const auto y_seg_name = prefix + "y";
  y_jnt.name = y_seg_name + jnt_suffix;
  y_jnt.type = Joint::TransAxis;
  y_jnt.axis(Vector::UnitY());
  Segment y_seg(y_seg_name, y_jnt);
  tree.addSegment(y_seg, x_seg_name);

  // Z
  Joint z_jnt;
  const auto z_seg_name = prefix + "z";
  z_jnt.name = z_seg_name + jnt_suffix;
  z_jnt.type = Joint::TransAxis;
  z_jnt.axis(Vector::UnitZ());
  Segment z_seg(z_seg_name, z_jnt);
  tree.addSegment(z_seg, y_seg_name);

  // Yaw
  Joint yaw_jnt;
  const auto yaw_seg_name = prefix + "yaw";
  yaw_jnt.name = yaw_seg_name + jnt_suffix;
  yaw_jnt.type = Joint::RotAxis;
  yaw_jnt.axis(Vector::UnitZ());
  Segment yaw_seg(yaw_seg_name, yaw_jnt);
  tree.addSegment(yaw_seg, z_seg_name);

  // Pitch
  Joint pitch_jnt;
  const auto pitch_seg_name = prefix + "pitch";
  pitch_jnt.name = pitch_seg_name + jnt_suffix;
  pitch_jnt.type = Joint::RotAxis;
  pitch_jnt.axis(Vector::UnitY());
  Segment pitch_seg(pitch_seg_name, pitch_jnt);
  tree.addSegment(pitch_seg, yaw_seg_name);

  // Roll
  Joint roll_jnt;
  const auto roll_seg_name = prefix + "roll";
  roll_jnt.name = roll_seg_name + jnt_suffix;
  roll_jnt.type = Joint::RotAxis;
  roll_jnt.axis(Vector::UnitX());
  Segment roll_seg(roll_seg_name, roll_jnt);
  tree.addSegment(roll_seg, pitch_seg_name);

  // Base
  Joint base_jnt;
  base_jnt.name = base_name;
  base_jnt.type = Joint::Fixed;
  Segment base_seg(base_name, base_jnt);
  tree.addSegment(base_seg, roll_seg_name);

  return tree;
}

void Tree::addSegment(const Segment& segment, const string& hook_name)
{
  if (tobas_std::contains(segments_, segment.name()))
    throw runtime_error("'" + segment.name() + "' already exists in the tree.");

  const auto parent = segments_.find(hook_name);
  if (parent == segments_.end())
    throw runtime_error("'" + hook_name + "' does not exist in the tree.");

  // Insert new element
  const auto q_nr = segment.getJoint().type != Joint::Fixed ? nj_ : 0;
  const auto retval =
    segments_.insert(make_pair(segment.name(), TreeElement(segment, parent, q_nr)));

  // check if insertion succeeded
  if (!retval.second)
    throw runtime_error("Failed to insert '" + segment.name() + "' into the tree.");

  // add iterator to new element in parents children list
  parent->second.children.push_back(retval.first);

  // increase number of segments
  ++ns_;

  // increase number of joints
  if (segment.getJoint().type != Joint::Fixed)
    ++nj_;
}

void Tree::addChain(const Chain& chain, const string& hook_name)
{
  auto parent_name = hook_name;
  for (size_t i = 0; i < chain.getNrOfSegments(); ++i)
  {
    addSegment(chain.getSegment(i), parent_name);
    parent_name = chain.getSegment(i).name();
  }
}

void Tree::addTree(const Tree& tree, const string& hook_name)
{
  return addTreeRecursive(tree.getRootSegment(), hook_name);
}

void Tree::addTreeRecursive(const SegmentMap::const_iterator& seg, const string& hook_name)
{
  for (size_t i = 0; i < seg->second.children.size(); ++i)
  {
    const auto& child = seg->second.children[i];
    addSegment(child->second.segment, hook_name);
    addTreeRecursive(child, child->first);
  }
}

void Tree::getChain(const string& chain_root, const string& chain_tip, Chain& chain) const
{
  // clear chain
  chain.clear();

  // walk down from chain_root and chain_tip to the seg of the tree
  vector<SegmentMap::key_type> parents_chain_root, parents_chain_tip;
  for (auto s = getSegment(chain_root); s != segments_.end(); s = s->second.parent)
  {
    parents_chain_root.push_back(s->first);
    if (s->first == root_name_)
      break;
  }
  if (parents_chain_root.empty() || parents_chain_root.back() != root_name_)
    throw runtime_error("'" + chain_root + "' is not found in the tree.");

  for (auto s = getSegment(chain_tip); s != segments_.end(); s = s->second.parent)
  {
    parents_chain_tip.push_back(s->first);
    if (s->first == root_name_)
      break;
  }
  if (parents_chain_tip.empty() || parents_chain_tip.back() != root_name_)
    throw runtime_error("'" + chain_tip + "' is not found in the tree.");

  // remove common part of segment lists
  auto last_segment = root_name_;
  while (!parents_chain_root.empty() && !parents_chain_tip.empty()
         && parents_chain_root.back() == parents_chain_tip.back())
  {
    last_segment = parents_chain_root.back();
    parents_chain_root.pop_back();
    parents_chain_tip.pop_back();
  }
  parents_chain_root.push_back(last_segment);

  // add the segments from the seg to the common frame
  for (size_t s = 0; s < parents_chain_root.size() - 1; ++s)
  {
    const auto& seg = getSegment(parents_chain_root[s])->second.segment;
    const auto f_tip = seg.pose(0).inverse();
    auto jnt = seg.getJoint();
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
}

void Tree::getSubTree(const string& segment_name, Tree& tree) const
{
  // check if segment_name exists
  const auto seg = segments_.find(segment_name);
  if (seg == segments_.end())
    throw runtime_error("'" + segment_name + "' is not found in the tree.");

  // init the tree, segment_name is the new seg.
  tree = Tree(seg->first);
  tree.addTreeRecursive(seg, segment_name);
}

ostream& operator<<(ostream& os, const Tree& arg)
{
  for (const auto& it : arg.segments_)
  {
    const auto& seg_name = it.first;
    const auto& elem = it.second;
    os << elem.q_nr << ": " << seg_name << endl;
  }

  return os;
}
}  // namespace tobas_kdl
