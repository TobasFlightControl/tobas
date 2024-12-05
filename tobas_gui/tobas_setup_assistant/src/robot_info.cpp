#include <iostream>
#include <urdf_parser/urdf_parser.h>

#include <tobas_kdl/kdl_parser.hpp>
#include <tobas_ros2_tools/xacro.hpp>

#include "tobas_setup_assistant/robot_info.hpp"
#include "tobas_setup_assistant/common.hpp"

using namespace std;

namespace gui
{
namespace setup_assistant
{
RobotInfo::RobotInfo() : axis_solver_(tree_)
{
}

bool RobotInfo::loadFromPath(const string& path)
{
  // Parse URDF
  if (!ros2::xacro(path, urdf_text_))
    return false;

  // Parse URDF
  urdf_ = urdf::parseURDF(urdf_text_);
  if (urdf_ == nullptr)
  {
    cerr << "Failed to parse URDF." << endl;
    return false;
  }

  // Load KDL tree
  if (!kdl::treeFromUrdfModel(*urdf_, tree_))
  {
    cerr << "Failed to load KDL tree." << endl;
    return false;
  }

  // Update KDL objects
  axis_solver_.updateInternalDataStructures();
  q_zeros_ = kdl::JntArray::Zero(tree_.getNrOfJoints());

  Q_EMIT loaded();
  return true;
}

const std::string& RobotInfo::urdfText() const
{
  return urdf_text_;
}

urdf::ModelInterfaceConstSharedPtr RobotInfo::urdf() const
{
  return urdf_;
}

const kdl::Tree& RobotInfo::tree() const
{
  return tree_;
}

const string& RobotInfo::robotName() const
{
  return urdf_->getName();
}

bool RobotInfo::isJntAxisAlwaysCollinear(const string& seg_name, const kdl::Vector& tar_axis)
{
  const auto seg_it = tree_.getSegment(seg_name);

  // 問題なくルートリンクまで遡れた場合はtrue．
  if (seg_it == tree_.getRootSegment())
    return true;

  // ある関節角に対し，チェーンを構成する全てのジョイント軸が目標と平行であることが必要十分条件．
  // つまり，可動関節で且つジョイント軸が目標と平行でないリンクが存在する場合はfalse．
  const auto& joint = seg_it->second.segment.joint();
  if (joint.type != kdl::Joint::FIXED)
  {
    if (axis_solver_.JntToCart(q_zeros_, seg_name) < 0)
    {
      cerr << "Failed to get the joint axis of " << seg_name << ": " << axis_solver_.errorMessage() << endl;
      return false;
    }
    const auto& cur_axis = axis_solver_.getAxis();
    if (cur_axis.argument(tar_axis) > kJntAxisCollinearTol)
      return false;
  }

  // 親リンクについて調べる
  const auto& par_name = seg_it->second.parent->first;
  return isJntAxisAlwaysCollinear(par_name, tar_axis);
}

tobas::rotor_axis_t RobotInfo::rotorAxisType(const string& seg_name)
{
  if (isJntAxisAlwaysCollinear(seg_name, kdl::Vector::UnitX()))
    return tobas::rotor_axis_t::X_POSITIVE;
  else if (isJntAxisAlwaysCollinear(seg_name, kdl::Vector::UnitZ()))
    return tobas::rotor_axis_t::Z_POSITIVE;
  else
    return tobas::rotor_axis_t::UNKNOWN;
}
}  // namespace setup_assistant
}  // namespace gui
