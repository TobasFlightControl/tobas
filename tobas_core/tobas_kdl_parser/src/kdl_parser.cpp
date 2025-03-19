#include <iostream>
#include <urdf_parser/urdf_parser.h>

#include <tobas_kdl_conversions/kdl_urdf.hpp>

#include "../include/tobas_kdl_parser/kdl_parser.hpp"

using namespace std;

namespace kdl
{
/* Recursive function to walk through tree. */
void addChildrenToTree(const urdf::LinkConstSharedPtr& root, Tree& tree)
{
  // Construct the KDL joint
  const auto joint = jointUrdfToKdl(*root->parent_joint);

  // Construct the tip frame
  const auto f_tip = poseUrdfToKdl(root->parent_joint->parent_to_joint_origin_transform);

  // Construct the optional inertia
  RigidBodyInertia inertia;
  if (root->inertial)
    inertiaUrdfToKdl(*root->inertial, inertia);

  // construct the KDL segment
  const Segment segment(root->name, joint, f_tip, inertia);

  // Add segment to tree
  tree.addSegment(segment, root->parent_joint->parent_link_name);

  // Recurslively add all children
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
  if (!robot_model)
  {
    cerr << "Failed to generate robot model." << endl;
    return false;
  }
  return treeFromUrdfModel(*robot_model, tree);
}

bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, Tree& tree)
{
  const auto root_link = robot_model.getRoot();
  if (!root_link)
  {
    cerr << "Failed to get root link." << endl;
    return false;
  }

  tree = Tree(root_link->name);

  // Warn if root link has inertia. tobas_kdl does not support this
  if (root_link->inertial)
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
