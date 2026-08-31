// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/robot_model.hpp"

#include <set>

#include <rclcpp/logging.hpp>

#include "tobas_rviz_plugin/logger.hpp"
#include "tobas_rviz_plugin/order_robot_model_items.hpp"

namespace tobas
{
namespace rviz
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::rviz::getLogger("tobas.robot_model");
}

using DescMap = std::map<
  JointModel::ConstSharedPtr,
  std::pair<
    std::set<LinkModel::ConstSharedPtr, OrderLinksByIndex>,
    std::set<JointModel::ConstSharedPtr, OrderJointsByIndex>>>;

void computeDescendantsHelper(
  const JointModel::ConstSharedPtr& joint,
  std::vector<JointModel::ConstSharedPtr>& parents,
  std::set<JointModel::ConstSharedPtr>& seen,
  DescMap& descendants)
{
  if (!joint) {
    return;
  }
  if (seen.find(joint) != seen.end()) {
    return;
  }
  seen.insert(joint);

  for (const auto& parent : parents) {
    descendants[parent].second.insert(joint);
  }

  const auto lm = joint->getChildLinkModel();
  if (!lm) {
    return;
  }

  for (const auto& parent : parents) {
    descendants[parent].first.insert(lm);
  }
  descendants[joint].first.insert(lm);

  parents.push_back(joint);
  const auto& ch = lm->getChildJointModels();
  for (const auto& child_joint_model : ch) {
    computeDescendantsHelper(child_joint_model, parents, seen, descendants);
  }
  const auto& mim = joint->getMimicRequests();
  for (const auto& mimic_joint_model : mim) {
    computeDescendantsHelper(mimic_joint_model, parents, seen, descendants);
  }
  parents.pop_back();
}

void computeCommonRootsHelper(const JointModel::ConstSharedPtr& joint, std::vector<int>& common_roots, int size)
{
  if (!joint) {
    return;
  }
  const auto lm = joint->getChildLinkModel();
  if (!lm) {
    return;
  }

  const auto& ch = lm->getChildJointModels();
  for (size_t i = 0; i < ch.size(); ++i) {
    const auto& a = ch[i]->getDescendantJointModels();
    for (size_t j = i + 1; j < ch.size(); ++j) {
      const auto& b = ch[j]->getDescendantJointModels();
      for (const auto& m : b) {
        common_roots[ch[i]->getJointIndex() * size + m->getJointIndex()] =
          common_roots[ch[i]->getJointIndex() + m->getJointIndex() * size] = joint->getJointIndex();
      }
      for (const auto& k : a) {
        common_roots[k->getJointIndex() * size + ch[j]->getJointIndex()] =
          common_roots[k->getJointIndex() + ch[j]->getJointIndex() * size] = joint->getJointIndex();
        for (const auto& m : b) {
          common_roots[k->getJointIndex() * size + m->getJointIndex()] =
            common_roots[k->getJointIndex() + m->getJointIndex() * size] = joint->getJointIndex();
        }
      }
    }
    computeCommonRootsHelper(ch[i], common_roots, size);
  }
}

Eigen::Isometry3d urdfPose2Isometry3d(const urdf::Pose& pose)
{
  const Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  const Eigen::Isometry3d af(Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) * q);
  return af;
}
}  // namespace

RobotModel::RobotModel(const urdf::ModelInterfaceSharedPtr& urdf_model)
{
  urdf_ = urdf_model;
  buildModel(*urdf_model);
}

const std::string& RobotModel::getModelFrame() const
{
  return model_frame_;
}

const urdf::ModelInterfaceSharedPtr& RobotModel::getURDF() const
{
  return urdf_;
}

JointModel::ConstSharedPtr RobotModel::getRootJoint() const
{
  return root_joint_;
}

bool RobotModel::hasJointModel(const std::string& name) const
{
  return joint_model_map_.find(name) != joint_model_map_.end();
}

JointModel::ConstSharedPtr RobotModel::getJointModel(const std::string& name) const
{
  const auto it = joint_model_map_.find(name);
  if (it == joint_model_map_.end()) {
    RCLCPP_ERROR(getLogger(), "Joint '%s' not found in model '%s'.", name.c_str(), model_name_.c_str());
    return nullptr;
  }
  return it->second;
}

JointModel::ConstSharedPtr RobotModel::getJointOfVariable(int variable_index) const
{
  return joints_of_variable_[variable_index];
}

size_t RobotModel::getJointModelCount() const
{
  return joint_model_vector_.size();
}

LinkModel::ConstSharedPtr RobotModel::getRootLink() const
{
  return root_link_;
}

LinkModel::ConstSharedPtr RobotModel::getLinkModel(const std::string& name) const
{
  const auto it = link_model_map_.find(name);
  if (it != link_model_map_.end()) {
    return it->second;
  }
  RCLCPP_ERROR(getLogger(), "Link '%s' not found in model '%s'.", name.c_str(), model_name_.c_str());
  return nullptr;
}

size_t RobotModel::getLinkModelCount() const
{
  return link_model_vector_.size();
}

void RobotModel::getVariableDefaultPositions(std::vector<double>& values) const
{
  values.resize(variable_count_);
  for (size_t i = 0; i < active_joint_model_vector_.size(); ++i) {
    active_joint_model_vector_[i]->getVariableDefaultPositions(values.data() + active_joint_model_start_index_[i]);
  }
  updateMimicJoints(values.data());
}

size_t RobotModel::getVariableCount() const
{
  return variable_count_;
}

size_t RobotModel::getVariableIndex(const std::string& variable) const
{
  const auto it = joint_variables_index_map_.find(variable);
  if (it == joint_variables_index_map_.end()) {
    throw std::runtime_error("Variable '" + variable + "' is not known to model '" + model_name_ + "'.");
  }
  return it->second;
}

JointModel::ConstSharedPtr
RobotModel::getCommonRoot(const JointModel::ConstSharedPtr& a, const JointModel::ConstSharedPtr& b) const
{
  if (!a) {
    return b;
  }
  if (!b) {
    return a;
  }
  return joint_model_vector_[common_joint_roots_[a->getJointIndex() * joint_model_vector_.size() + b->getJointIndex()]];
}

void RobotModel::updateMimicJoints(double* values) const
{
  for (const auto& mimic_joint : mimic_joints_) {
    const auto src = mimic_joint->getMimic()->getFirstVariableIndex();
    const auto dest = mimic_joint->getFirstVariableIndex();
    values[dest] = values[src] * mimic_joint->getMimicFactor() + mimic_joint->getMimicOffset();
  }
}

void RobotModel::buildModel(const urdf::ModelInterface& urdf_model)
{
  model_name_ = urdf_model.getName();
  RCLCPP_INFO(getLogger(), "Loading robot model '%s'...", model_name_.c_str());

  if (urdf_model.getRoot()) {
    const auto root_link = urdf_model.getRoot();
    model_frame_ = root_link->name;

    RCLCPP_DEBUG(getLogger(), "... building kinematic chain.");
    root_joint_ = buildRecursive(nullptr, root_link);
    if (root_joint_) {
      root_link_ = link_model_map_.at(root_joint_->getChildLinkModel()->getName());
    }
    RCLCPP_DEBUG(getLogger(), "... building mimic joints.");
    buildMimic(urdf_model);

    RCLCPP_DEBUG(getLogger(), "... computing joint indexing.");
    buildJointInfo();
  }
  else {
    RCLCPP_WARN(getLogger(), "No root link found.");
  }
}

void RobotModel::buildMimic(const urdf::ModelInterface& urdf_model)
{
  // Compute mimic joints.
  for (const auto& joint_model : joint_model_vector_) {
    const auto jm = urdf_model.getJoint(joint_model->getName());
    if (jm) {
      if (jm->mimic) {
        const auto jit = joint_model_map_.find(jm->mimic->joint_name);
        if (jit != joint_model_map_.end()) {
          if (joint_model->getVariableCount() == jit->second->getVariableCount()) {
            joint_model->setMimic(jit->second, jm->mimic->multiplier, jm->mimic->offset);
          }
          else {
            RCLCPP_ERROR(
              getLogger(),
              "Joint '%s' cannot mimic joint '%s' because they have different number of DOF",
              joint_model->getName().c_str(),
              jm->mimic->joint_name.c_str());
          }
        }
        else {
          RCLCPP_ERROR(
            getLogger(),
            "Joint '%s' cannot mimic unknown joint '%s'",
            joint_model->getName().c_str(),
            jm->mimic->joint_name.c_str());
        }
      }
    }
  }

  // In case we have a joint that mimics a joint that already mimics another joint, we can simplify things:
  bool change = true;
  while (change) {
    change = false;
    for (const auto& joint_model : joint_model_vector_) {
      if (joint_model->getMimic()) {
        if (joint_model->getMimic()->getMimic()) {
          joint_model->setMimic(
            joint_model->getMimic()->getMimic(),
            joint_model->getMimicFactor() * joint_model->getMimic()->getMimicFactor(),
            joint_model->getMimicOffset() + joint_model->getMimicFactor() * joint_model->getMimic()->getMimicOffset());
          change = true;
        }
        if (joint_model == joint_model->getMimic()) {
          RCLCPP_ERROR(getLogger(), "Cycle found in joint that mimic each other. Ignoring all mimic joints.");
          for (const auto& joint_model_recal : joint_model_vector_) {
            joint_model_recal->setMimic(nullptr, 0.0, 0.0);
          }
          change = false;
          break;
        }
      }
    }
  }

  // Build mimic requests.
  for (const auto& joint_model : joint_model_vector_) {
    if (joint_model->getMimic()) {
      joint_model_map_.at(joint_model->getMimic()->getName())->addMimicRequest(joint_model);
      mimic_joints_.push_back(joint_model);
    }
  }
}

void RobotModel::buildJointInfo()
{
  // Construct additional maps for easy access by name.
  variable_count_ = 0;

  for (const auto& joint : joint_model_vector_) {
    const auto& name_order = joint->getVariableNames();

    // Compute index map.
    if (!name_order.empty()) {
      for (size_t j = 0; j < name_order.size(); ++j) {
        joint_variables_index_map_[name_order[j]] = variable_count_ + j;
        joints_of_variable_.push_back(joint);
      }
      if (!joint->getMimic()) {
        active_joint_model_start_index_.push_back(variable_count_);
        active_joint_model_vector_.push_back(joint);
      }

      joint_variables_index_map_[joint->getName()] = variable_count_;

      // Compute variable count.
      const auto vc = joint->getVariableCount();
      variable_count_ += vc;
    }
  }

  computeDescendants();
  computeCommonRoots();  // Must be called _after_ list of descendants was computed.
}

void RobotModel::computeDescendants()
{
  // Compute the list of descendants for all joints.
  std::vector<JointModel::ConstSharedPtr> parents;
  std::set<JointModel::ConstSharedPtr> seen;

  DescMap descendants;
  computeDescendantsHelper(root_joint_, parents, seen, descendants);
  for (const auto& descendant : descendants) {
    const auto jm = joint_model_map_.at(descendant.first->getName());
    for (const auto& jt : descendant.second.second) {
      jm->addDescendantJointModel(jt);
    }
    for (const auto& jt : descendant.second.first) {
      jm->addDescendantLinkModel(jt);
    }
  }
}

void RobotModel::computeCommonRoots()
{
  // Compute common roots for all pairs of joints.
  // There are 3 cases of pairs (X, Y):
  //    X != Y && X and Y are not descendants of one another.
  //    X == Y
  //    X != Y && X and Y are descendants of one another.

  // By default, the common root is always the global root.
  common_joint_roots_.resize(joint_model_vector_.size() * joint_model_vector_.size(), 0);

  // Look at all descendants recursively.
  // For two sibling nodes A, B, both children of X,
  // all the pairs of respective descendants of A and B have X as the common root.
  computeCommonRootsHelper(root_joint_, common_joint_roots_, joint_model_vector_.size());

  for (const auto& joint_model : joint_model_vector_) {
    // The common root of a joint and itself is the same joint:
    common_joint_roots_[joint_model->getJointIndex() * (1 + joint_model_vector_.size())] = joint_model->getJointIndex();

    // A node N and one of its descendants have as common root the node N itself:
    const auto& d = joint_model->getDescendantJointModels();
    for (const auto& descendant_joint_model : d) {
      common_joint_roots_
        [descendant_joint_model->getJointIndex() * joint_model_vector_.size() + joint_model->getJointIndex()] =
          common_joint_roots_
            [descendant_joint_model->getJointIndex() + joint_model->getJointIndex() * joint_model_vector_.size()] =
              joint_model->getJointIndex();
    }
  }
}

JointModel::SharedPtr
RobotModel::buildRecursive(const LinkModel::SharedPtr& parent, const urdf::LinkConstSharedPtr& urdf_link)
{
  // Construct the joint.
  const auto joint = constructJointModel(urdf_link);

  if (!joint) {
    return nullptr;
  }

  // Bookkeeping for the joint
  joint_model_vector_.push_back(joint);
  joint_model_map_[joint->getName()] = joint;

  // Construct the link.
  const auto link = constructLinkModel(urdf_link);
  joint->setChildLinkModel(link);
  link->setParentLinkModel(parent);

  // Bookkeeping for the link
  link_model_map_[joint->getChildLinkModel()->getName()] = link;
  link_model_vector_.push_back(link);
  link->setParentJointModel(joint);

  // Recursively build child links (and joints).
  for (const auto& child_link : urdf_link->child_links) {
    const auto jm = buildRecursive(link, child_link);
    if (jm) {
      link->addChildJointModel(jm);
    }
  }
  return joint;
}

JointModel::SharedPtr RobotModel::constructJointModel(const urdf::LinkConstSharedPtr& child_link)
{
  JointModel::SharedPtr new_joint_model;
  const auto parent_joint = child_link->parent_joint;
  const auto joint_index = joint_model_vector_.size();
  const auto first_variable_index = joint_model_vector_.empty() ? 0 :
                                                                  joint_model_vector_.back()->getFirstVariableIndex() +
                                                                    joint_model_vector_.back()->getVariableCount();

  // if parent_joint exists, must be the root link transform.
  if (parent_joint) {
    switch (parent_joint->type) {
      case urdf::Joint::REVOLUTE: {
        const auto j = std::make_shared<RevoluteJointModel>(parent_joint->name, joint_index, first_variable_index);
        j->setAxis(Eigen::Vector3d(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z));
        new_joint_model = j;
      } break;
      case urdf::Joint::CONTINUOUS: {
        const auto j = std::make_shared<RevoluteJointModel>(parent_joint->name, joint_index, first_variable_index);
        j->setAxis(Eigen::Vector3d(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z));
        new_joint_model = j;
      } break;
      case urdf::Joint::PRISMATIC: {
        const auto j = std::make_shared<PrismaticJointModel>(parent_joint->name, joint_index, first_variable_index);
        j->setAxis(Eigen::Vector3d(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z));
        new_joint_model = j;
      } break;
      case urdf::Joint::FLOATING:
        new_joint_model = std::make_shared<FloatingJointModel>(parent_joint->name, joint_index, first_variable_index);
        break;
      case urdf::Joint::PLANAR:
        new_joint_model = std::make_shared<PlanarJointModel>(parent_joint->name, joint_index, first_variable_index);
        break;
      case urdf::Joint::FIXED:
        new_joint_model = std::make_shared<FixedJointModel>(parent_joint->name, joint_index, first_variable_index);
        break;
      case urdf::Joint::UNKNOWN:
      default:
        RCLCPP_ERROR(getLogger(), "Unknown joint type: %d", static_cast<int>(parent_joint->type));
        break;
    }
  }
  else  // If parent_joint passed in as null, then we're at root of URDF model.
  {
    RCLCPP_INFO(getLogger(), "No root/virtual joint specified in URDF. Assuming fixed joint.");
    new_joint_model = std::make_shared<FixedJointModel>("ASSUMED_FIXED_ROOT_JOINT", joint_index, first_variable_index);
  }

  return new_joint_model;
}

LinkModel::SharedPtr RobotModel::constructLinkModel(const urdf::LinkConstSharedPtr& urdf_link)
{
  const auto link_index = link_model_vector_.size();
  const auto new_link_model = std::make_shared<LinkModel>(urdf_link->name, link_index);

  if (urdf_link->parent_joint) {
    new_link_model->setJointOriginTransform(
      urdfPose2Isometry3d(urdf_link->parent_joint->parent_to_joint_origin_transform));
  }

  return new_link_model;
}

}  // namespace rviz
}  // namespace tobas
