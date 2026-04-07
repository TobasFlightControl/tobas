// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/robot_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <set>

#include <geometric_shapes/shape_operations.h>
#include <rclcpp/logger.hpp>

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

typedef std::map<
  const JointModel*,
  std::pair<std::set<const LinkModel*, OrderLinksByIndex>, std::set<const JointModel*, OrderJointsByIndex>>>
  DescMap;

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

  for (const JointModel* parent : parents) {
    descendants[parent].second.insert(joint);
  }

  const LinkModel* lm = joint->getChildLinkModel();
  if (!lm) {
    return;
  }

  for (const JointModel* parent : parents) {
    descendants[parent].first.insert(lm);
  }
  descendants[joint].first.insert(lm);

  parents.push_back(joint);
  const std::vector<const JointModel*>& ch = lm->getChildJointModels();
  for (const JointModel* child_joint_model : ch) {
    computeDescendantsHelper(child_joint_model, parents, seen, descendants);
  }
  const std::vector<const JointModel*>& mim = joint->getMimicRequests();
  for (const JointModel* mimic_joint_model : mim) {
    computeDescendantsHelper(mimic_joint_model, parents, seen, descendants);
  }
  parents.pop_back();
}

void computeCommonRootsHelper(const JointModel* joint, std::vector<int>& common_roots, int size)
{
  if (!joint) {
    return;
  }
  const LinkModel* lm = joint->getChildLinkModel();
  if (!lm) {
    return;
  }

  const std::vector<const JointModel*>& ch = lm->getChildJointModels();
  for (size_t i = 0; i < ch.size(); ++i) {
    const std::vector<const JointModel*>& a = ch[i]->getDescendantJointModels();
    for (size_t j = i + 1; j < ch.size(); ++j) {
      const std::vector<const JointModel*>& b = ch[j]->getDescendantJointModels();
      for (const JointModel* m : b) {
        common_roots[ch[i]->getJointIndex() * size + m->getJointIndex()] =
          common_roots[ch[i]->getJointIndex() + m->getJointIndex() * size] = joint->getJointIndex();
      }
      for (const JointModel* k : a) {
        common_roots[k->getJointIndex() * size + ch[j]->getJointIndex()] =
          common_roots[k->getJointIndex() + ch[j]->getJointIndex() * size] = joint->getJointIndex();
        for (const JointModel* m : b) {
          common_roots[k->getJointIndex() * size + m->getJointIndex()] =
            common_roots[k->getJointIndex() + m->getJointIndex() * size] = joint->getJointIndex();
        }
      }
    }
    computeCommonRootsHelper(ch[i], common_roots, size);
  }
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
  for (auto joint_model : joint_model_vector_) {
    delete joint_model;
  }
  for (auto link_model : link_model_vector_) {
    delete link_model;
  }
}

const JointModel* RobotModel::getRootJoint() const
{
  return root_joint_;
}

const LinkModel* RobotModel::getRootLink() const
{
  return root_link_;
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
    const urdf::Link* root_link_ptr = urdf_model.getRoot().get();
    model_frame_ = root_link_ptr->name;

    RCLCPP_DEBUG(getLogger(), "... building kinematic chain");
    root_joint_ = buildRecursive(nullptr, root_link_ptr);
    if (root_joint_) {
      root_link_ = root_joint_->getChildLinkModel();
    }
    RCLCPP_DEBUG(getLogger(), "... building mimic joints");
    buildMimic(urdf_model);

    RCLCPP_DEBUG(getLogger(), "... computing joint indexing");
    buildJointInfo();

    if (link_models_with_collision_geometry_vector_.empty()) {
      RCLCPP_WARN(getLogger(), "No geometry is associated to any robot links");
    }
  }
  else {
    RCLCPP_WARN(getLogger(), "No root link found");
  }
}

void RobotModel::computeCommonRoots()
{
  // compute common roots for all pairs of joints;
  // there are 3 cases of pairs (X, Y):
  //    X != Y && X and Y are not descendants of one another
  //    X == Y
  //    X != Y && X and Y are descendants of one another

  // by default, the common root is always the global root;
  common_joint_roots_.resize(joint_model_vector_.size() * joint_model_vector_.size(), 0);

  // look at all descendants recursively; for two sibling nodes A, B, both children of X, all the pairs of respective
  // descendants of A and B
  // have X as the common root.
  computeCommonRootsHelper(root_joint_, common_joint_roots_, joint_model_vector_.size());

  for (const JointModel* joint_model : joint_model_vector_) {
    // the common root of a joint and itself is the same joint:
    common_joint_roots_[joint_model->getJointIndex() * (1 + joint_model_vector_.size())] = joint_model->getJointIndex();

    // a node N and one of its descendants have as common root the node N itself:
    const std::vector<const JointModel*>& d = joint_model->getDescendantJointModels();
    for (const JointModel* descendant_joint_model : d) {
      common_joint_roots_
        [descendant_joint_model->getJointIndex() * joint_model_vector_.size() + joint_model->getJointIndex()] =
          common_joint_roots_
            [descendant_joint_model->getJointIndex() + joint_model->getJointIndex() * joint_model_vector_.size()] =
              joint_model->getJointIndex();
    }
  }
}

void RobotModel::computeDescendants()
{
  // compute the list of descendants for all joints
  std::vector<const JointModel*> parents;
  std::set<const JointModel*> seen;

  DescMap descendants;
  computeDescendantsHelper(root_joint_, parents, seen, descendants);
  for (std::pair<
         const JointModel* const,
         std::pair<std::set<const LinkModel*, OrderLinksByIndex>, std::set<const JointModel*, OrderJointsByIndex>>>&
         descendant : descendants) {
    JointModel* jm = const_cast<JointModel*>(descendant.first);
    for (const JointModel* jt : descendant.second.second) {
      jm->addDescendantJointModel(jt);
    }
    for (const LinkModel* jt : descendant.second.first) {
      jm->addDescendantLinkModel(jt);
    }
  }
}

void RobotModel::buildJointInfo()
{
  // construct additional maps for easy access by name
  variable_count_ = 0;
  active_joint_model_start_index_.reserve(joint_model_vector_.size());
  variable_names_.reserve(joint_model_vector_.size());
  joints_of_variable_.reserve(joint_model_vector_.size());

  for (const auto& joint : joint_model_vector_) {
    const std::vector<std::string>& name_order = joint->getVariableNames();

    // compute index map
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

      if (joint->getType() == JointModel::REVOLUTE && static_cast<const RevoluteJointModel*>(joint)->isContinuous()) {
        continuous_joint_model_vector_.push_back(joint);
      }

      joint_variables_index_map_[joint->getName()] = variable_count_;

      // compute variable count
      size_t vc = joint->getVariableCount();
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
    for (auto& tf_base : associated_transforms) {
      link_considered[tf_base.first->getLinkIndex()] = true;
      for (auto& tf_target : associated_transforms) {
        if (&tf_base != &tf_target) {
          const_cast<LinkModel*>(tf_base.first)  // regain write access to base LinkModel*
            ->addAssociatedFixedTransform(tf_target.first, tf_base.second.inverse() * tf_target.second);
        }
      }
    }
  }

  computeDescendants();
  computeCommonRoots();  // must be called _after_ list of descendants was computed
}

void RobotModel::buildMimic(const urdf::ModelInterface& urdf_model)
{
  // compute mimic joints
  for (JointModel* joint_model : joint_model_vector_) {
    const urdf::Joint* jm = urdf_model.getJoint(joint_model->getName()).get();
    if (jm) {
      if (jm->mimic) {
        JointModelMap::const_iterator jit = joint_model_map_.find(jm->mimic->joint_name);
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

  // in case we have a joint that mimics a joint that already mimics another joint, we can simplify things:
  bool change = true;
  while (change) {
    change = false;
    for (JointModel* joint_model : joint_model_vector_) {
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
          for (JointModel* joint_model_recal : joint_model_vector_) {
            joint_model_recal->setMimic(nullptr, 0., 0.);
          }
          change = false;
          break;
        }
      }
    }
  }
  // build mimic requests
  for (JointModel* joint_model : joint_model_vector_) {
    if (joint_model->getMimic()) {
      const_cast<JointModel*>(joint_model->getMimic())->addMimicRequest(joint_model);
      mimic_joints_.push_back(joint_model);
    }
  }
}

JointModel* RobotModel::buildRecursive(LinkModel* parent, const urdf::Link* urdf_link)
{
  // construct the joint
  JointModel* joint = constructJointModel(urdf_link);

  if (!joint) {
    return nullptr;
  }

  // bookkeeping for the joint
  joint_model_vector_.push_back(joint);
  joint_model_map_[joint->getName()] = joint;
  joint_model_vector_const_.push_back(joint);
  joint_model_names_vector_.push_back(joint->getName());
  joint->setParentLinkModel(parent);

  // construct the link
  LinkModel* link = constructLinkModel(urdf_link);
  joint->setChildLinkModel(link);
  link->setParentLinkModel(parent);

  // bookkeeping for the link
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

  // recursively build child links (and joints)
  for (const urdf::LinkSharedPtr& child_link : urdf_link->child_links) {
    JointModel* jm = buildRecursive(link, child_link.get());
    if (jm) {
      link->addChildJointModel(jm);
    }
  }
  return joint;
}

namespace
{
// construct bounds for 1DOF joint
inline VariableBounds jointBoundsFromURDF(const urdf::Joint* urdf_joint)
{
  VariableBounds b;
  if (urdf_joint->safety) {
    b.position_bounded_ = true;
    b.min_position_ = urdf_joint->safety->soft_lower_limit;
    b.max_position_ = urdf_joint->safety->soft_upper_limit;
    if (urdf_joint->limits) {
      if (urdf_joint->limits->lower > b.min_position_) {
        b.min_position_ = urdf_joint->limits->lower;
      }
      if (urdf_joint->limits->upper < b.max_position_) {
        b.max_position_ = urdf_joint->limits->upper;
      }
    }
  }
  else {
    if (urdf_joint->limits) {
      b.position_bounded_ = true;
      b.min_position_ = urdf_joint->limits->lower;
      b.max_position_ = urdf_joint->limits->upper;
    }
  }
  if (urdf_joint->limits) {
    b.max_velocity_ = std::abs(urdf_joint->limits->velocity);
    b.min_velocity_ = -b.max_velocity_;
    b.velocity_bounded_ = b.max_velocity_ > std::numeric_limits<double>::epsilon();
  }
  return b;
}
}  // namespace

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
        RevoluteJointModel* j = new RevoluteJointModel(parent_joint->name, joint_index, first_variable_index);
        j->setVariableBounds(j->getName(), jointBoundsFromURDF(parent_joint));
        j->setContinuous(false);
        j->setAxis(Eigen::Vector3d(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z));
        new_joint_model = j;
      } break;
      case urdf::Joint::CONTINUOUS: {
        RevoluteJointModel* j = new RevoluteJointModel(parent_joint->name, joint_index, first_variable_index);
        j->setVariableBounds(j->getName(), jointBoundsFromURDF(parent_joint));
        j->setContinuous(true);
        j->setAxis(Eigen::Vector3d(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z));
        new_joint_model = j;
      } break;
      case urdf::Joint::PRISMATIC: {
        PrismaticJointModel* j = new PrismaticJointModel(parent_joint->name, joint_index, first_variable_index);
        j->setVariableBounds(j->getName(), jointBoundsFromURDF(parent_joint));
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
  else  // if parent_joint passed in as null, then we're at root of URDF model
  {
    RCLCPP_INFO(getLogger(), "No root/virtual joint specified in URDF. Assuming fixed joint.");
    new_joint_model = new FixedJointModel("ASSUMED_FIXED_ROOT_JOINT", joint_index, first_variable_index);
  }

  if (new_joint_model) {
    new_joint_model->setDistanceFactor(new_joint_model->getStateSpaceDimension());
  }

  return new_joint_model;
}

namespace
{
inline Eigen::Isometry3d urdfPose2Isometry3d(const urdf::Pose& pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Isometry3d af(Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) * q);
  return af;
}
}  // namespace

LinkModel* RobotModel::constructLinkModel(const urdf::Link* urdf_link)
{
  auto link_index = link_model_vector_.size();
  LinkModel* new_link_model = new LinkModel(urdf_link->name, link_index);

  const std::vector<urdf::CollisionSharedPtr>& col_array =
    urdf_link->collision_array.empty() ? std::vector<urdf::CollisionSharedPtr>(1, urdf_link->collision) :
                                         urdf_link->collision_array;

  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Isometry3d poses;

  for (const urdf::CollisionSharedPtr& col : col_array) {
    if (col && col->geometry) {
      shapes::ShapeConstPtr s = constructShape(col->geometry.get());
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
    for (const urdf::VisualSharedPtr& vis : vis_array) {
      if (vis && vis->geometry) {
        warn_about_missing_collision = true;
      }
    }
  }
  if (warn_about_missing_collision) {
    RCLCPP_WARN_STREAM(
      getLogger(),  // TODO(henningkayser): use child namespace "empty_collision_geometry"
      "Link " << urdf_link->name
              << " has visual geometry but no collision geometry. "
                 "Collision geometry will be left empty. "
                 "Fix your URDF file by explicitly specifying collision geometry.");
  }

  new_link_model->setGeometry(shapes, poses);

  // figure out visual mesh (try visual urdf tag first, collision tag otherwise
  if (urdf_link->visual && urdf_link->visual->geometry) {
    if (urdf_link->visual->geometry->type == urdf::Geometry::MESH) {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(urdf_link->visual->geometry.get());
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
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(urdf_link->collision->geometry.get());
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
    case urdf::Geometry::SPHERE:
      new_shape = new shapes::Sphere(static_cast<const urdf::Sphere*>(geom)->radius);
      break;
    case urdf::Geometry::BOX: {
      urdf::Vector3 dim = static_cast<const urdf::Box*>(geom)->dim;
      new_shape = new shapes::Box(dim.x, dim.y, dim.z);
    } break;
    case urdf::Geometry::CYLINDER:
      new_shape = new shapes::Cylinder(
        static_cast<const urdf::Cylinder*>(geom)->radius, static_cast<const urdf::Cylinder*>(geom)->length);
      break;
    case urdf::Geometry::MESH: {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty()) {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        shapes::Mesh* m = shapes::createMeshFromResource(mesh->filename, scale);
        new_shape = m;
      }
    } break;
    default:
      RCLCPP_ERROR(getLogger(), "Unknown geometry type: %d", static_cast<int>(geom->type));
      break;
  }

  return shapes::ShapePtr(new_shape);
}

bool RobotModel::hasJointModel(const std::string& name) const
{
  return joint_model_map_.find(name) != joint_model_map_.end();
}

const JointModel* RobotModel::getJointModel(const std::string& name) const
{
  JointModelMap::const_iterator it = joint_model_map_.find(name);
  if (it != joint_model_map_.end()) {
    return it->second;
  }
  RCLCPP_ERROR(getLogger(), "Joint '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
  return nullptr;
}

const JointModel* RobotModel::getJointModel(size_t index) const
{
  if (index >= joint_model_vector_.size()) {
    RCLCPP_ERROR(getLogger(), "Joint index '%li' out of bounds of joints in model '%s'", index, model_name_.c_str());
    return nullptr;
  }
  assert(joint_model_vector_[index]->getJointIndex() == index);
  return joint_model_vector_[index];
}

JointModel* RobotModel::getJointModel(const std::string& name)
{
  JointModelMap::const_iterator it = joint_model_map_.find(name);
  if (it != joint_model_map_.end()) {
    return it->second;
  }
  RCLCPP_ERROR(getLogger(), "Joint '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
  return nullptr;
}

const LinkModel* RobotModel::getLinkModel(const std::string& name, bool* has_link) const
{
  return const_cast<RobotModel*>(this)->getLinkModel(name, has_link);
}

const LinkModel* RobotModel::getLinkModel(size_t index) const
{
  if (index >= link_model_vector_.size()) {
    RCLCPP_ERROR(getLogger(), "Link index '%li' out of bounds of links in model '%s'", index, model_name_.c_str());
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
    RCLCPP_ERROR(getLogger(), "Link '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
  }
  return nullptr;
}

void RobotModel::computeFixedTransforms(
  const LinkModel* link,
  const Eigen::Isometry3d& transform,
  LinkTransformMap& associated_transforms)
{
  associated_transforms[link] = transform * link->getJointOriginTransform();
  for (size_t i = 0; i < link->getChildJointModels().size(); ++i) {
    if (link->getChildJointModels()[i]->getType() == JointModel::FIXED) {
      computeFixedTransforms(
        link->getChildJointModels()[i]->getChildLinkModel(),
        transform * link->getJointOriginTransform(),
        associated_transforms);
    }
  }
}

void RobotModel::updateMimicJoints(double* values) const
{
  for (const JointModel* mimic_joint : mimic_joints_) {
    int src = mimic_joint->getMimic()->getFirstVariableIndex();
    int dest = mimic_joint->getFirstVariableIndex();
    values[dest] = values[src] * mimic_joint->getMimicFactor() + mimic_joint->getMimicOffset();
  }
}

void RobotModel::getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values) const
{
  for (size_t i = 0; i < active_joint_model_vector_.size(); ++i) {
    active_joint_model_vector_[i]->getVariableRandomPositions(rng, values + active_joint_model_start_index_[i]);
  }
  updateMimicJoints(values);
}

void RobotModel::getVariableRandomPositions(
  random_numbers::RandomNumberGenerator& rng,
  std::map<std::string, double>& values) const
{
  std::vector<double> tmp(variable_count_);
  getVariableRandomPositions(rng, &tmp.front());
  values.clear();
  for (size_t i = 0; i < variable_names_.size(); ++i) {
    values[variable_names_[i]] = tmp[i];
  }
}

void RobotModel::getVariableDefaultPositions(double* values) const
{
  for (size_t i = 0; i < active_joint_model_vector_.size(); ++i) {
    active_joint_model_vector_[i]->getVariableDefaultPositions(values + active_joint_model_start_index_[i]);
  }
  updateMimicJoints(values);
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

size_t RobotModel::getVariableIndex(const std::string& variable) const
{
  VariableIndexMap::const_iterator it = joint_variables_index_map_.find(variable);
  if (it == joint_variables_index_map_.end()) {
    throw std::runtime_error("Variable '" + variable + "' is not known to model '" + model_name_ + '\'');
  }
  return it->second;
}
}  // namespace tobas
