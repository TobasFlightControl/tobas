// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/robot_model.hpp"

#include <set>

#include <geometric_shapes/shape_operations.h>
#include <rclcpp/logging.hpp>

#include "tobas_rviz_plugin/logger.hpp"
#include "tobas_rviz_plugin/order_robot_model_items.hpp"

namespace tobas
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::getLogger("tobas.robot_model");
}

using DescMap = std::map<
  const JointModel*,
  std::pair<std::set<const LinkModel*, OrderLinksByIndex>, std::set<const JointModel*, OrderJointsByIndex>>>;

void computeDescendantsHelper(
  const JointModel* joint,
  std::vector<const JointModel*>& parents,
  std::set<const JointModel*>& seen,
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

void computeCommonRootsHelper(const JointModel* joint, std::vector<int>& common_roots, int size)
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
  root_joint_ = nullptr;
  urdf_ = urdf_model;
  buildModel(*urdf_model);
}

RobotModel::~RobotModel()
{
  for (const auto& joint_model : joint_model_vector_) {
    delete joint_model;
  }
  for (const auto& link_model : link_model_vector_) {
    delete link_model;
  }
}

const std::string& RobotModel::getModelFrame() const
{
  return model_frame_;
}

const urdf::ModelInterfaceSharedPtr& RobotModel::getURDF() const
{
  return urdf_;
}

const JointModel* RobotModel::getRootJoint() const
{
  return root_joint_;
}

bool RobotModel::hasJointModel(const std::string& name) const
{
  return joint_model_map_.find(name) != joint_model_map_.end();
}

const JointModel* RobotModel::getJointModel(const std::string& name) const
{
  const auto it = joint_model_map_.find(name);
  if (it == joint_model_map_.end()) {
    RCLCPP_ERROR(getLogger(), "Joint '%s' not found in model '%s'.", name.c_str(), model_name_.c_str());
    return nullptr;
  }
  return it->second;
}

const JointModel* RobotModel::getJointModel(size_t index) const
{
  if (index >= joint_model_vector_.size()) {
    RCLCPP_ERROR(getLogger(), "Joint index '%li' out of bounds of joints in model '%s'.", index, model_name_.c_str());
    return nullptr;
  }
  assert(joint_model_vector_[index]->getJointIndex() == index);
  return joint_model_vector_[index];
}

JointModel* RobotModel::getJointModel(const std::string& name)
{
  const auto it = joint_model_map_.find(name);
  if (it != joint_model_map_.end()) {
    return it->second;
  }
  RCLCPP_ERROR(getLogger(), "Joint '%s' not found in model '%s'.", name.c_str(), model_name_.c_str());
  return nullptr;
}

const std::vector<const JointModel*>& RobotModel::getSingleDOFJointModels() const
{
  return single_dof_joints_;
}

const std::vector<const JointModel*>& RobotModel::getMultiDOFJointModels() const
{
  return multi_dof_joints_;
}

const JointModel* RobotModel::getJointOfVariable(int variable_index) const
{
  return joints_of_variable_[variable_index];
}

const JointModel* RobotModel::getJointOfVariable(const std::string& variable) const
{
  return joints_of_variable_[getVariableIndex(variable)];
}

size_t RobotModel::getJointModelCount() const
{
  return joint_model_vector_.size();
}

const LinkModel* RobotModel::getRootLink() const
{
  return root_link_;
}

const LinkModel* RobotModel::getLinkModel(const std::string& name, bool* has_link) const
{
  return const_cast<RobotModel*>(this)->getLinkModel(name, has_link);
}

const LinkModel* RobotModel::getLinkModel(size_t index) const
{
  if (index >= link_model_vector_.size()) {
    RCLCPP_ERROR(getLogger(), "Link index '%li' out of bounds of links in model '%s'.", index, model_name_.c_str());
    return nullptr;
  }
  assert(link_model_vector_[index]->getLinkIndex() == index);
  return link_model_vector_[index];
}

LinkModel* RobotModel::getLinkModel(const std::string& name, bool* has_link)
{
  if (has_link) {
    *has_link = true;  // Start out optimistic
  }
  const auto it = link_model_map_.find(name);
  if (it != link_model_map_.end()) {
    return it->second;
  }

  if (has_link) {
    *has_link = false;  // Report failure via argument
  }
  else {  // Otherwise print error
    RCLCPP_ERROR(getLogger(), "Link '%s' not found in model '%s'.", name.c_str(), model_name_.c_str());
  }
  return nullptr;
}

size_t RobotModel::getLinkModelCount() const
{
  return link_model_vector_.size();
}

size_t RobotModel::getLinkGeometryCount() const
{
  return link_geometry_count_;
}

void RobotModel::getVariableDefaultPositions(double* values) const
{
  for (size_t i = 0; i < active_joint_model_vector_.size(); ++i) {
    active_joint_model_vector_[i]->getVariableDefaultPositions(values + active_joint_model_start_index_[i]);
  }
  updateMimicJoints(values);
}

void RobotModel::getVariableDefaultPositions(std::vector<double>& values) const
{
  values.resize(variable_count_);
  getVariableDefaultPositions(&values.front());
}

void RobotModel::getVariableDefaultPositions(std::map<std::string, double>& values) const
{
  std::vector<double> tmp(variable_count_);
  getVariableDefaultPositions(&tmp.front());
  values.clear();
  for (size_t i = 0; i < variable_names_.size(); ++i) {
    values[variable_names_[i]] = tmp[i];
  }
}

size_t RobotModel::getVariableCount() const
{
  return variable_count_;
}

const std::vector<std::string>& RobotModel::getVariableNames() const
{
  return variable_names_;
}

size_t RobotModel::getVariableIndex(const std::string& variable) const
{
  const auto it = joint_variables_index_map_.find(variable);
  if (it == joint_variables_index_map_.end()) {
    throw std::runtime_error("Variable '" + variable + "' is not known to model '" + model_name_ + "'.");
  }
  return it->second;
}

const JointModel* RobotModel::getCommonRoot(const JointModel* a, const JointModel* b) const
{
  if (!a) {
    return b;
  }
  if (!b) {
    return a;
  }
  return joint_model_vector_[common_joint_roots_[a->getJointIndex() * joint_model_vector_.size() + b->getJointIndex()]];
}

void RobotModel::computeFixedTransforms(
  const LinkModel* link,
  const Eigen::Isometry3d& transform,
  LinkTransformMap& associated_transforms)
{
  associated_transforms[link] = transform * link->getJointOriginTransform();
  for (size_t i = 0; i < link->getChildJointModels().size(); ++i) {
    if (link->getChildJointModels()[i]->getType() == JointModel::kFixed) {
      computeFixedTransforms(
        link->getChildJointModels()[i]->getChildLinkModel(),
        transform * link->getJointOriginTransform(),
        associated_transforms);
    }
  }
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
  root_joint_ = nullptr;
  root_link_ = nullptr;
  link_geometry_count_ = 0;
  variable_count_ = 0;
  model_name_ = urdf_model.getName();
  RCLCPP_INFO(getLogger(), "Loading robot model '%s'...", model_name_.c_str());

  if (urdf_model.getRoot()) {
    const auto root_link_ptr = urdf_model.getRoot().get();
    model_frame_ = root_link_ptr->name;

    RCLCPP_DEBUG(getLogger(), "... building kinematic chain.");
    root_joint_ = buildRecursive(nullptr, root_link_ptr);
    if (root_joint_) {
      root_link_ = root_joint_->getChildLinkModel();
    }
    RCLCPP_DEBUG(getLogger(), "... building mimic joints.");
    buildMimic(urdf_model);

    RCLCPP_DEBUG(getLogger(), "... computing joint indexing.");
    buildJointInfo();

    if (link_models_with_collision_geometry_vector_.empty()) {
      RCLCPP_WARN(getLogger(), "No geometry is associated to any robot links.");
    }
  }
  else {
    RCLCPP_WARN(getLogger(), "No root link found.");
  }
}

void RobotModel::buildMimic(const urdf::ModelInterface& urdf_model)
{
  // Compute mimic joints
  for (const auto& joint_model : joint_model_vector_) {
    const auto jm = urdf_model.getJoint(joint_model->getName()).get();
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

  // Build mimic requests
  for (const auto& joint_model : joint_model_vector_) {
    if (joint_model->getMimic()) {
      const_cast<JointModel*>(joint_model->getMimic())->addMimicRequest(joint_model);
      mimic_joints_.push_back(joint_model);
    }
  }
}

void RobotModel::buildJointInfo()
{
  // Construct additional maps for easy access by name
  variable_count_ = 0;
  active_joint_model_start_index_.reserve(joint_model_vector_.size());
  variable_names_.reserve(joint_model_vector_.size());
  joints_of_variable_.reserve(joint_model_vector_.size());

  for (const auto& joint : joint_model_vector_) {
    const auto& name_order = joint->getVariableNames();

    // Compute index map
    if (!name_order.empty()) {
      for (size_t j = 0; j < name_order.size(); ++j) {
        joint_variables_index_map_[name_order[j]] = variable_count_ + j;
        variable_names_.push_back(name_order[j]);
        joints_of_variable_.push_back(joint);
      }
      if (!joint->getMimic()) {
        active_joint_model_start_index_.push_back(variable_count_);
        active_joint_model_vector_.push_back(joint);
        active_joint_model_names_vector_.push_back(joint->getName());
        active_joint_model_vector_const_.push_back(joint);
      }

      if (joint->getType() == JointModel::kRevolute && static_cast<const RevoluteJointModel*>(joint)->isContinuous()) {
        continuous_joint_model_vector_.push_back(joint);
      }

      joint_variables_index_map_[joint->getName()] = variable_count_;

      // Compute variable count
      const auto vc = joint->getVariableCount();
      variable_count_ += vc;
      if (vc == 1) {
        single_dof_joints_.push_back(joint);
      }
      else {
        multi_dof_joints_.push_back(joint);
      }
    }
  }

  std::vector<bool> link_considered(link_model_vector_.size(), false);
  for (const LinkModel* link : link_model_vector_) {
    if (link_considered[link->getLinkIndex()]) {
      continue;
    }

    LinkTransformMap associated_transforms;
    computeFixedTransforms(link, link->getJointOriginTransform().inverse(), associated_transforms);
    for (const auto& tf_base : associated_transforms) {
      link_considered[tf_base.first->getLinkIndex()] = true;
      for (const auto& tf_target : associated_transforms) {
        if (&tf_base != &tf_target) {
          const_cast<LinkModel*>(tf_base.first)  // Regain write access to base LinkModel
            ->addAssociatedFixedTransform(tf_target.first, tf_base.second.inverse() * tf_target.second);
        }
      }
    }
  }

  computeDescendants();
  computeCommonRoots();  // Must be called _after_ list of descendants was computed
}

void RobotModel::computeDescendants()
{
  // Compute the list of descendants for all joints
  std::vector<const JointModel*> parents;
  std::set<const JointModel*> seen;

  DescMap descendants;
  computeDescendantsHelper(root_joint_, parents, seen, descendants);
  for (const auto& descendant : descendants) {
    const auto jm = const_cast<JointModel*>(descendant.first);
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
  //    X != Y && X and Y are not descendants of one another
  //    X == Y
  //    X != Y && X and Y are descendants of one another

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

JointModel* RobotModel::buildRecursive(LinkModel* parent, const urdf::Link* urdf_link)
{
  // Construct the joint
  const auto joint = constructJointModel(urdf_link);

  if (!joint) {
    return nullptr;
  }

  // Bookkeeping for the joint
  joint_model_vector_.push_back(joint);
  joint_model_map_[joint->getName()] = joint;
  joint_model_vector_const_.push_back(joint);
  joint_model_names_vector_.push_back(joint->getName());
  joint->setParentLinkModel(parent);

  // Construct the link
  const auto link = constructLinkModel(urdf_link);
  joint->setChildLinkModel(link);
  link->setParentLinkModel(parent);

  // Bookkeeping for the link
  link_model_map_[joint->getChildLinkModel()->getName()] = link;
  link_model_vector_.push_back(link);
  link_model_vector_const_.push_back(link);
  link_model_names_vector_.push_back(link->getName());
  if (!link->getShapes().empty()) {
    link_models_with_collision_geometry_vector_.push_back(link);
    link_model_names_with_collision_geometry_vector_.push_back(link->getName());
    link->setFirstCollisionBodyTransformIndex(link_geometry_count_);
    link_geometry_count_ += link->getShapes().size();
  }
  link->setParentJointModel(joint);

  // Recursively build child links (and joints)
  for (const auto& child_link : urdf_link->child_links) {
    const auto jm = buildRecursive(link, child_link.get());
    if (jm) {
      link->addChildJointModel(jm);
    }
  }
  return joint;
}

JointModel* RobotModel::constructJointModel(const urdf::Link* child_link)
{
  JointModel* new_joint_model = nullptr;
  const auto parent_joint = child_link->parent_joint ? child_link->parent_joint.get() : nullptr;
  const auto joint_index = joint_model_vector_.size();
  const auto first_variable_index = joint_model_vector_.empty() ? 0 :
                                                                  joint_model_vector_.back()->getFirstVariableIndex() +
                                                                    joint_model_vector_.back()->getVariableCount();

  // if parent_joint exists, must be the root link transform
  if (parent_joint) {
    switch (parent_joint->type) {
      case urdf::Joint::REVOLUTE: {
        const auto j = new RevoluteJointModel(parent_joint->name, joint_index, first_variable_index);
        j->setContinuous(false);
        j->setAxis(Eigen::Vector3d(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z));
        new_joint_model = j;
      } break;
      case urdf::Joint::CONTINUOUS: {
        const auto j = new RevoluteJointModel(parent_joint->name, joint_index, first_variable_index);
        j->setContinuous(true);
        j->setAxis(Eigen::Vector3d(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z));
        new_joint_model = j;
      } break;
      case urdf::Joint::PRISMATIC: {
        const auto j = new PrismaticJointModel(parent_joint->name, joint_index, first_variable_index);
        j->setAxis(Eigen::Vector3d(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z));
        new_joint_model = j;
      } break;
      case urdf::Joint::FLOATING:
        new_joint_model = new FloatingJointModel(parent_joint->name, joint_index, first_variable_index);
        break;
      case urdf::Joint::PLANAR:
        new_joint_model = new PlanarJointModel(parent_joint->name, joint_index, first_variable_index);
        break;
      case urdf::Joint::FIXED:
        new_joint_model = new FixedJointModel(parent_joint->name, joint_index, first_variable_index);
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
    new_joint_model = new FixedJointModel("ASSUMED_FIXED_ROOT_JOINT", joint_index, first_variable_index);
  }

  return new_joint_model;
}

LinkModel* RobotModel::constructLinkModel(const urdf::Link* urdf_link)
{
  const auto link_index = link_model_vector_.size();
  const auto new_link_model = new LinkModel(urdf_link->name, link_index);

  const std::vector<urdf::CollisionSharedPtr>& col_array =
    urdf_link->collision_array.empty() ? std::vector<urdf::CollisionSharedPtr>(1, urdf_link->collision) :
                                         urdf_link->collision_array;

  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Isometry3d poses;

  for (const auto& col : col_array) {
    if (col && col->geometry) {
      const auto s = constructShape(col->geometry.get());
      if (s) {
        shapes.push_back(s);
        poses.push_back(urdfPose2Isometry3d(col->origin));
      }
    }
  }

  // Should we warn that old (melodic) behaviour has changed, not copying visual to collision geometries anymore?
  bool warn_about_missing_collision = false;
  if (shapes.empty()) {
    const auto& vis_array = urdf_link->visual_array.empty() ? std::vector<urdf::VisualSharedPtr>{ urdf_link->visual } :
                                                              urdf_link->visual_array;
    for (const auto& vis : vis_array) {
      if (vis && vis->geometry) {
        warn_about_missing_collision = true;
      }
    }
  }
  if (warn_about_missing_collision) {
    RCLCPP_WARN_STREAM(
      getLogger(),
      "Link " << urdf_link->name
              << " has visual geometry but no collision geometry. "
                 "Collision geometry will be left empty. "
                 "Fix your URDF file by explicitly specifying collision geometry.");
  }

  new_link_model->setGeometry(shapes, poses);

  // Figure out visual mesh (try visual urdf tag first, collision tag otherwise
  if (urdf_link->visual && urdf_link->visual->geometry) {
    if (urdf_link->visual->geometry->type == urdf::Geometry::MESH) {
      const auto mesh = static_cast<const urdf::Mesh*>(urdf_link->visual->geometry.get());
      if (!mesh->filename.empty()) {
        new_link_model->setVisualMesh(
          mesh->filename,
          urdfPose2Isometry3d(urdf_link->visual->origin),
          Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z));
      }
    }
  }
  else if (urdf_link->collision && urdf_link->collision->geometry) {
    if (urdf_link->collision->geometry->type == urdf::Geometry::MESH) {
      const auto mesh = static_cast<const urdf::Mesh*>(urdf_link->collision->geometry.get());
      if (!mesh->filename.empty()) {
        new_link_model->setVisualMesh(
          mesh->filename,
          urdfPose2Isometry3d(urdf_link->collision->origin),
          Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z));
      }
    }
  }

  if (urdf_link->parent_joint) {
    new_link_model->setJointOriginTransform(
      urdfPose2Isometry3d(urdf_link->parent_joint->parent_to_joint_origin_transform));
  }

  return new_link_model;
}

shapes::ShapePtr RobotModel::constructShape(const urdf::Geometry* geom)
{
  shapes::Shape* new_shape = nullptr;
  switch (geom->type) {
    case urdf::Geometry::SPHERE: {
      new_shape = new shapes::Sphere(static_cast<const urdf::Sphere*>(geom)->radius);
      break;
    }
    case urdf::Geometry::BOX: {
      const auto dim = static_cast<const urdf::Box*>(geom)->dim;
      new_shape = new shapes::Box(dim.x, dim.y, dim.z);
      break;
    }
    case urdf::Geometry::CYLINDER: {
      new_shape = new shapes::Cylinder(
        static_cast<const urdf::Cylinder*>(geom)->radius, static_cast<const urdf::Cylinder*>(geom)->length);
      break;
    }
    case urdf::Geometry::MESH: {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty()) {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        shapes::Mesh* m = shapes::createMeshFromResource(mesh->filename, scale);
        new_shape = m;
      }
      break;
    }
    default: {
      RCLCPP_ERROR(getLogger(), "Unknown geometry type: %d", static_cast<int>(geom->type));
      break;
    }
  }

  return shapes::ShapePtr(new_shape);
}
}  // namespace tobas
