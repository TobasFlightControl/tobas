#include "tobas_uadf/model.hpp"

#include <iostream>

namespace uadf
{
Model::Model()
{
}

void Model::clear()
{
  urdf.reset();

  thrusts.clear();
  control_surfaces.clear();
  tilts.clear();
}

bool Model::valid() const
{
  // URDFが存在する
  if (!urdf) {
    std::cerr << "URDF is null." << std::endl;
    return false;
  }

  // "thrust"はエンドジョイントでなければならない
  for (const auto& [thrust_joint_name, thrust] : thrusts) {
    const auto thrust_joint = urdf->getJoint(thrust_joint_name);
    if (!thrust_joint) {
      std::cerr << "Thrust joint \"" << thrust_joint_name << "\" does not exist." << std::endl;
      return false;
    }

    const auto& thrust_link_name = thrust_joint->child_link_name;
    const auto thrust_link = urdf->getLink(thrust_link_name);
    if (!thrust_link) {
      std::cerr << "Thrust link \"" << thrust_link_name << "\" does not exist." << std::endl;
      return false;
    }

    if (!thrust_link->child_joints.empty()) {
      std::cerr << "Thrust link \"" << thrust_link_name << "\" must be an end link." << std::endl;
      return false;
    }
  }

  // "cs"はエンドジョイントでなければならない
  for (const auto& [cs_joint_name, cs] : control_surfaces) {
    const auto cs_joint = urdf->getJoint(cs_joint_name);
    if (!cs_joint) {
      std::cerr << "CS joint \"" << cs_joint_name << "\" does not exist." << std::endl;
      return false;
    }

    const auto& cs_link_name = cs_joint->child_link_name;
    const auto cs_link = urdf->getLink(cs_link_name);
    if (!cs_link) {
      std::cerr << "CS link \"" << cs_link_name << "\" does not exist." << std::endl;
      return false;
    }

    if (!cs_link->child_joints.empty()) {
      std::cerr << "CS link \"" << cs_link_name << "\" must be an end link." << std::endl;
      return false;
    }
  }

  // "tilt"の次のジョイントは"thrust"が1つのみである
  for (const auto& [tilt_joint_name, tilt] : tilts) {
    const auto tilt_joint = urdf->getJoint(tilt_joint_name);
    if (!tilt_joint) {
      std::cerr << "Tilt joint \"" << tilt_joint_name << "\" does not exist." << std::endl;
      return false;
    }

    const auto& tilt_link_name = tilt_joint->child_link_name;
    const auto tilt_link = urdf->getLink(tilt_link_name);
    if (!tilt_link) {
      std::cerr << "Tilt link \"" << tilt_link_name << "\" does not exist." << std::endl;
      return false;
    }

    if (tilt_link->child_joints.size() != 1) {
      std::cerr << "Tilt link \"" << tilt_link_name << "\" must have one child joint." << std::endl;
      return false;
    }
    const auto& child_joint = tilt_link->child_joints.front();

    if (!thrusts.contains(child_joint->name)) {
      std::cerr << "The joint type following \"" << tilt_joint_name << "\" must be \"thrust\"." << std::endl;
      return false;
    }
  }

  return true;
}
}  // namespace uadf
