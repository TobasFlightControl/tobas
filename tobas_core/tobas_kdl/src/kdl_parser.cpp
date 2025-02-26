#include <iostream>
#include <urdf_parser/urdf_parser.h>

#include "../include/tobas_kdl/kdl_parser.hpp"
#include "../include/tobas_kdl/conversion/urdf_kdl.hpp"

using namespace std;

namespace kdl
{
/* Recursive function to walk through tree. */
void addChildrenToTree(const urdf::LinkConstSharedPtr& root, Tree& tree)
{
  // constructs the optional inertia
  RigidBodyInertia inertia(0);
  if (root->inertial != nullptr)
    inertia = toKdl(*root->inertial);

  // constructs the kdl joint
  const auto jnt = toKdl(*root->parent_joint);

  // construct the kdl segment
  const Segment sgm(root->name, jnt, toKdl(root->parent_joint->parent_to_joint_origin_transform), inertia);

  // add segment to tree
  tree.addSegment(sgm, root->parent_joint->parent_link_name);

  // recurslively add all children
  for (const auto& child : root->child_links)
    addChildrenToTree(child, tree);
}

bool treeFromFile(const string& file, Tree& tree)
{
  const auto robot_model = urdf::parseURDFFile(file);
  return treeFromUrdfModel(*robot_model, tree);
}

bool treeFromString(const string& xml, Tree& tree)
{
  const auto robot_model = urdf::parseURDF(xml);
  if (robot_model == nullptr)
  {
    cerr << "Failed to generate robot model." << endl;
    return false;
  }
  return treeFromUrdfModel(*robot_model, tree);
}

bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, Tree& tree)
{
  const auto root_link = robot_model.getRoot();
  if (root_link == nullptr)
  {
    cerr << "Failed to get root link." << endl;
    return false;
  }

  tree = Tree(root_link->name);

  // Warn if root link has inertia. tobas_kdl does not support this
  if (root_link->inertial != nullptr)
  {
    cerr << "The root link " << root_link->name << " has an inertia specified in the URDF, "
         << "but tobas_kdl does not support a root link with an inertia. "
         << "As a workaround, you can add an extra dummy link to your URDF." << endl;
  }

  // Add all children
  for (const auto& child : root_link->child_links)
    addChildrenToTree(child, tree);

  return true;
}
}  // namespace kdl
