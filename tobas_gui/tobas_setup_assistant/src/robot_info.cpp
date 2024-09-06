#include <iostream>
#include <hardware_interface/component_parser.hpp>

#include <tobas_kdl/kdl_parser.hpp>
#include <tobas_ros2_tools/xacro.hpp>

#include "tobas_setup_assistant/robot_info.hpp"

using namespace std;

namespace gui
{
namespace setup_assistant
{
RobotInfo::RobotInfo(rclcpp::Node::SharedPtr node)
  : node_(node), rsp_client_(node, "robot_state_publisher"), axis_solver_(tree_)
{
}

bool RobotInfo::loadFromPath(const std::string& path)
{
  // Parse URDF
  string urdf_content;
  if (!ros2::xacro(path, urdf_content))
    return false;

  // Update RSP parameter
  if (!rsp_client_.service_is_ready())
  {
    RCLCPP_ERROR(node_->get_logger(), "Robot state publisher is not ready.");
    return false;
  }
  const auto res = rsp_client_.set_parameters({ rclcpp::Parameter("robot_description", urdf_content) }).at(0);
  if (!res.successful)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to set robot description parameter of RSP: " << res.reason);
    return false;
  }

  // Load KDL tree
  if (!kdl::treeFromString(urdf_content, tree_))
  {
    cerr << "Failed to load KDL tree." << endl;
    return false;
  }

  // Load hardware information
  const auto hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf_content);
  if (hardware_infos.size() != 1)
  {
    cerr << "The number of hardware information objects must be 1." << endl;
    return false;
  }
  hardware_ = hardware_infos.at(0);

  // Update KDL objects
  axis_solver_.updateInternalDataStructures();
  q_zeros_ = kdl::JntArray::Zero(tree_.getNrOfJoints());

  Q_EMIT loaded();
  return true;
}

const kdl::Tree& RobotInfo::tree() const
{
  return tree_;
}

const hardware_interface::HardwareInfo& RobotInfo::hardware() const
{
  return hardware_;
}

const std::string& RobotInfo::robotName() const
{
  return hardware_.name;
}

hw_interface::type_t RobotInfo::hardwareInterface(const string& jnt_name) const
{
  for (const auto& transmission : hardware_.transmissions)
  {
    for (const auto& joint : transmission.joints)
    {
      if (joint.name == jnt_name)
      {
        const auto& hi = transmission.type;
        if (hi == hw_interface::kPositionInterface)
        {
          return hw_interface::POSITION;
        }
        else if (hi == hw_interface::kVelocityInterface)
        {
          return hw_interface::VELOCITY;
        }
        else if (hi == hw_interface::kEffortInterface)
        {
          return hw_interface::EFFORT;
        }
        else
        {
          cerr << "Invalid hardware interface of joint " << jnt_name << ": " << hi << endl;
          return hw_interface::UNKNOWN;
        }
      }
    }
  }

  return hw_interface::NONE;
}

bool RobotInfo::isJntAxisAlwaysCollinear(const std::string& seg_name, const kdl::Vector& tar_axis)
{
  const auto seg_it = tree_.getSegment(seg_name);

  // 問題なくルートリンクまで遡れた場合はtrue．
  if (seg_it == tree_.getRootSegment())
    return true;

  // ある関節角に対し，チェーンを構成する全てのジョイント軸が目標と平行であることが必要十分条件．
  // つまり，可動関節で且つジョイント軸が目標と平行でないリンクが存在する場合はfalse．
  const auto& joint = seg_it->second.segment.joint();
  if (joint.type != kdl::Joint::Fixed)
  {
    if (!axis_solver_.JntToCart(q_zeros_, seg_name))
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

tobas::rotor_axis_t RobotInfo::rotorAxisType(const std::string& seg_name)
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
