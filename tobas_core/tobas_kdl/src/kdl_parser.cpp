#include <ros/ros.h>
#include <urdf/model.h>
#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>

#include <tobas_std_tools/console.hpp>

#include "../include/tobas_kdl/kdl_parser.hpp"

using namespace std;

namespace KDL
{
// construct vector
Vector toKdl(const urdf::Vector3& v)
{
  return Vector(v.x, v.y, v.z);
}

// construct rotation
Rotation toKdl(const urdf::Rotation& r)
{
  return Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
Frame toKdl(const urdf::Pose& p)
{
  return Frame(toKdl(p.rotation), toKdl(p.position));
}

// construct joint
Joint toKdl(const urdf::Joint& jnt)
{
  const Frame F_parent_jnt = toKdl(jnt.parent_to_joint_origin_transform);

  Joint res;
  res.name = jnt.name;

  if (jnt.type == urdf::Joint::FIXED)
  {
    res.type = Joint::Fixed;
  }
  else
  {
    if (jnt.type == urdf::Joint::REVOLUTE || jnt.type == urdf::Joint::CONTINUOUS)
      res.type = Joint::RotAxis;
    else if (jnt.type == urdf::Joint::PRISMATIC)
      res.type = Joint::TransAxis;
    else
      throw runtime_error("Unknown joint type of joint: " + jnt.name);

    res.origin = F_parent_jnt.p;
    res.axis(F_parent_jnt.M * toKdl(jnt.axis));

    if (jnt.dynamics != nullptr)
    {
      res.damping = jnt.dynamics->damping;
      res.friction = jnt.dynamics->friction;
    }

    if (jnt.limits != nullptr)
    {
      res.lower_limit = jnt.limits->lower;
      res.upper_limit = jnt.limits->upper;
      res.max_effort = jnt.limits->effort;
      res.max_velocity = jnt.limits->velocity;
    }
  }

  return res;
}

// construct inertia
RigidBodyInertia toKdl(const urdf::Inertial& i)
{
  const Frame origin = toKdl(i.origin);

  // the mass is frame independent
  const double& kdl_mass = i.mass;

  // kdl and urdf both specify the com position in the reference frame of the link
  const Vector& kdl_com = origin.p;

  // kdl specifies the inertia matrix in the reference frame of the link,
  // while the urdf specifies the inertia matrix in the inertia reference frame
  const RotationalInertia urdf_inertia(i.ixx, i.iyy, i.izz, i.ixy, i.ixz, i.iyz);

  // Rotation operators are not defined for rotational inertia,
  // so we use the RigidBodyInertia operators (with com = 0) as a workaround
  const RigidBodyInertia kdl_inertia_wrt_com_workaround =
    origin.M * RigidBodyInertia(0, Vector::Zero(), urdf_inertia);

  // Note that the RigidBodyInertia constructor takes the 3d inertia wrt the com
  // while the getRotationalInertia method returns the 3d inertia wrt the frame origin
  // (but having com = Vector::Zero() in kdl_inertia_wrt_com_workaround they match)
  const RotationalInertia kdl_inertia_wrt_com =
    kdl_inertia_wrt_com_workaround.getRotationalInertia();

  return RigidBodyInertia(kdl_mass, kdl_com, kdl_inertia_wrt_com);
}

// recursive function to walk through tree
void addChildrenToTree(const urdf::LinkConstSharedPtr& root, Tree& tree)
{
  PRINT_DEBUG_ONCE("KDL::addChildrenToTree");

  // constructs the optional inertia
  RigidBodyInertia inertia(0);
  if (root->inertial)
    inertia = toKdl(*root->inertial);

  // constructs the kdl joint
  const Joint jnt = toKdl(*root->parent_joint);

  // construct the kdl segment
  const Segment sgm(
    root->name, jnt, toKdl(root->parent_joint->parent_to_joint_origin_transform), inertia);

  // add segment to tree
  tree.addSegment(sgm, root->parent_joint->parent_link_name);

  // recurslively add all children
  for (const auto& child : root->child_links)
    addChildrenToTree(child, tree);
}

bool treeFromFile(const string& file, Tree& tree)
{
  PRINT_DEBUG("KDL::treeFromFile");

  const urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDFFile(file);
  return treeFromUrdfModel(*robot_model, tree);
}

bool treeFromParam(const string& param, Tree& tree)
{
  PRINT_DEBUG("KDL::treeFromParam");

  urdf::Model robot_model;
  if (!robot_model.initParam(param))
  {
    ROS_ERROR("Failed to generate robot model.");
    return false;
  }
  return treeFromUrdfModel(robot_model, tree);
}

bool treeFromString(const string& xml, Tree& tree)
{
  PRINT_DEBUG("KDL::treeFromString");

  const urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDF(xml);
  if (!robot_model)
  {
    ROS_ERROR("Failed to generate robot model.");
    return false;
  }
  return treeFromUrdfModel(*robot_model, tree);
}

bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, Tree& tree)
{
  PRINT_DEBUG("KDL::treeFromUrdfModel");

  if (!robot_model.getRoot())
  {
    ROS_ERROR("Failed to get root link.");
    return false;
  }

  tree = Tree(robot_model.getRoot()->name);

  // Warn if root link has inertia. KDL does not support this
  if (robot_model.getRoot()->inertial)
  {
    ROS_WARN_STREAM(
      "The root link " << robot_model.getRoot()->name << " has an inertia specified in the URDF, "
                       << "but KDL does not support a root link with an inertia. "
                       << "As a workaround, you can add an extra dummy link to your URDF.");
  }

  // Add all children
  for (const auto& child : robot_model.getRoot()->child_links)
    addChildrenToTree(child, tree);

  return true;
}
}  // namespace KDL
