// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/robot_state.hpp"

#include <functional>

#include <geometric_shapes/check_isometry.h>
#include <geometric_shapes/shape_operations.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "tobas_rviz_plugin/logger.hpp"

namespace ch = std::chrono;

namespace tobas
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::getLogger("tobas.robot_state");
}
}  // namespace

RobotState::RobotState(const RobotModel::ConstSharedPtr& robot_model)
  : robot_model_(robot_model)
  , has_velocity_(false)
  , has_acceleration_(false)
  , has_effort_(false)
  , dirty_link_transforms_(nullptr)
  , dirty_collision_body_transforms_(nullptr)
{
  dirty_link_transforms_ = robot_model_->getRootJoint();
  init();
}

void RobotState::init()
{
  variable_joint_transforms_.resize(robot_model_->getJointModelCount(), Eigen::Isometry3d::Identity());
  global_link_transforms_.resize(robot_model_->getLinkModelCount(), Eigen::Isometry3d::Identity());
  global_collision_body_transforms_.resize(robot_model_->getLinkGeometryCount(), Eigen::Isometry3d::Identity());
  dirty_joint_transforms_.resize(robot_model_->getJointModelCount(), 1);
  position_.resize(robot_model_->getVariableCount());
  velocity_.resize(robot_model_->getVariableCount());
  effort_or_acceleration_.resize(robot_model_->getVariableCount());
}

bool RobotState::checkJointTransforms(const JointModel* joint) const
{
  if (dirtyJointTransform(joint)) {
    RCLCPP_WARN(getLogger(), "Returning dirty joint transforms for joint '%s'", joint->getName().c_str());
    return false;
  }
  return true;
}

bool RobotState::checkLinkTransforms() const
{
  if (dirtyLinkTransforms()) {
    RCLCPP_WARN(getLogger(), "Returning dirty link transforms");
    return false;
  }
  return true;
}

bool RobotState::checkCollisionTransforms() const
{
  if (dirtyCollisionBodyTransforms()) {
    RCLCPP_WARN(getLogger(), "Returning dirty collision body transforms");
    return false;
  }
  return true;
}

void RobotState::markVelocity()
{
  if (!has_velocity_) {
    has_velocity_ = true;
    std::fill(velocity_.begin(), velocity_.end(), 0.);
  }
}

void RobotState::markAcceleration()
{
  if (!has_acceleration_) {
    has_acceleration_ = true;
    has_effort_ = false;
    std::fill(effort_or_acceleration_.begin(), effort_or_acceleration_.end(), 0.);
  }
}

void RobotState::markEffort()
{
  if (!has_effort_) {
    has_acceleration_ = false;
    has_effort_ = true;
    std::fill(effort_or_acceleration_.begin(), effort_or_acceleration_.end(), 0.);
  }
}

void RobotState::updateMimicJoint(const JointModel* joint)
{
  if (joint->getVariableCount() == 0) {
    return;
  }
  double v = position_[joint->getFirstVariableIndex()];
  for (const JointModel* jm : joint->getMimicRequests()) {
    position_[jm->getFirstVariableIndex()] = jm->getMimicFactor() * v + jm->getMimicOffset();
    markDirtyJointTransforms(jm);
  }
}

void RobotState::setVariablePositions(const double* position)
{
  // assume everything is in order in terms of array lengths (for efficiency reasons)
  memcpy(position_.data(), position, robot_model_->getVariableCount() * sizeof(double));

  // the full state includes mimic joint values, so no need to update mimic here

  // Since all joint values have potentially changed, we will need to recompute all transforms
  std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
  dirty_link_transforms_ = robot_model_->getRootJoint();
}

void RobotState::setVariablePositions(const std::map<std::string, double>& variable_map)
{
  for (const std::pair<const std::string, double>& it : variable_map) {
    const int index = robot_model_->getVariableIndex(it.first);
    position_[index] = it.second;
    const JointModel* jm = robot_model_->getJointOfVariable(index);
    markDirtyJointTransforms(jm);
    updateMimicJoint(jm);
  }
}

void RobotState::getMissingKeys(
  const std::map<std::string, double>& variable_map,
  std::vector<std::string>& missing_variables) const
{
  missing_variables.clear();
  const std::vector<std::string>& nm = robot_model_->getVariableNames();
  for (const std::string& variable_name : nm) {
    if (variable_map.find(variable_name) == variable_map.end()) {
      if (!robot_model_->getJointOfVariable(variable_name)->getMimic()) {
        missing_variables.push_back(variable_name);
      }
    }
  }
}

void RobotState::setVariablePositions(
  const std::map<std::string, double>& variable_map,
  std::vector<std::string>& missing_variables)
{
  setVariablePositions(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariablePositions(
  const std::vector<std::string>& variable_names,
  const std::vector<double>& variable_position)
{
  for (size_t i = 0; i < variable_names.size(); ++i) {
    const int index = robot_model_->getVariableIndex(variable_names[i]);
    position_[index] = variable_position[i];
    const JointModel* jm = robot_model_->getJointOfVariable(index);
    markDirtyJointTransforms(jm);
    updateMimicJoint(jm);
  }
}

void RobotState::setVariableVelocities(const std::map<std::string, double>& variable_map)
{
  markVelocity();
  for (const std::pair<const std::string, double>& it : variable_map) {
    velocity_[robot_model_->getVariableIndex(it.first)] = it.second;
  }
}

void RobotState::setVariableVelocities(
  const std::map<std::string, double>& variable_map,
  std::vector<std::string>& missing_variables)
{
  setVariableVelocities(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableVelocities(
  const std::vector<std::string>& variable_names,
  const std::vector<double>& variable_velocity)
{
  markVelocity();
  assert(variable_names.size() == variable_velocity.size());
  for (size_t i = 0; i < variable_names.size(); ++i) {
    velocity_[robot_model_->getVariableIndex(variable_names[i])] = variable_velocity[i];
  }
}

void RobotState::setVariableAccelerations(const std::map<std::string, double>& variable_map)
{
  markAcceleration();
  for (const std::pair<const std::string, double>& it : variable_map) {
    effort_or_acceleration_[robot_model_->getVariableIndex(it.first)] = it.second;
  }
}

void RobotState::setVariableAccelerations(
  const std::map<std::string, double>& variable_map,
  std::vector<std::string>& missing_variables)
{
  setVariableAccelerations(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableAccelerations(
  const std::vector<std::string>& variable_names,
  const std::vector<double>& variable_acceleration)
{
  markAcceleration();
  assert(variable_names.size() == variable_acceleration.size());
  for (size_t i = 0; i < variable_names.size(); ++i) {
    effort_or_acceleration_[robot_model_->getVariableIndex(variable_names[i])] = variable_acceleration[i];
  }
}

void RobotState::setVariableEffort(const std::map<std::string, double>& variable_map)
{
  markEffort();
  for (const std::pair<const std::string, double>& it : variable_map) {
    effort_or_acceleration_[robot_model_->getVariableIndex(it.first)] = it.second;
  }
}

void RobotState::setVariableEffort(
  const std::map<std::string, double>& variable_map,
  std::vector<std::string>& missing_variables)
{
  setVariableEffort(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableEffort(
  const std::vector<std::string>& variable_names,
  const std::vector<double>& variable_effort)
{
  markEffort();
  assert(variable_names.size() == variable_effort.size());
  for (size_t i = 0; i < variable_names.size(); ++i) {
    effort_or_acceleration_[robot_model_->getVariableIndex(variable_names[i])] = variable_effort[i];
  }
}

void RobotState::setJointPositions(const JointModel* joint, const double* position)
{
  if (joint->getVariableCount() == 0) {
    return;
  }
  memcpy(&position_.at(joint->getFirstVariableIndex()), position, joint->getVariableCount() * sizeof(double));
  markDirtyJointTransforms(joint);
  updateMimicJoint(joint);
}

void RobotState::setJointPositions(const JointModel* joint, const Eigen::Isometry3d& transform)
{
  if (joint->getVariableCount() == 0) {
    return;
  }
  joint->computeVariablePositions(transform, &position_.at(joint->getFirstVariableIndex()));
  markDirtyJointTransforms(joint);
  updateMimicJoint(joint);
}

void RobotState::setJointVelocities(const JointModel* joint, const double* velocity)
{
  if (joint->getVariableCount() == 0) {
    return;
  }
  has_velocity_ = true;
  memcpy(&velocity_.at(joint->getFirstVariableIndex()), velocity, joint->getVariableCount() * sizeof(double));
}

const double* RobotState::getJointPositions(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &position_.at(joint->getFirstVariableIndex());
}

const double* RobotState::getJointVelocities(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &velocity_.at(joint->getFirstVariableIndex());
}

const double* RobotState::getJointAccelerations(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &effort_or_acceleration_.at(joint->getFirstVariableIndex());
}

const double* RobotState::getJointEffort(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &effort_or_acceleration_.at(joint->getFirstVariableIndex());
}

void RobotState::setJointEfforts(const JointModel* joint, const double* effort)
{
  if (has_acceleration_) {
    RCLCPP_ERROR(getLogger(), "Unable to set joint efforts because array is being used for accelerations");
    return;
  }
  if (joint->getVariableCount() == 0) {
    return;
  }
  has_effort_ = true;

  memcpy(
    &effort_or_acceleration_.at(joint->getFirstVariableIndex()), effort, joint->getVariableCount() * sizeof(double));
}

void RobotState::setToDefaultValues()
{
  robot_model_->getVariableDefaultPositions(position_);  // mimic values are updated
  // set velocity & acceleration to 0
  std::fill(velocity_.begin(), velocity_.end(), 0);
  std::fill(effort_or_acceleration_.begin(), effort_or_acceleration_.end(), 0);
  std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
  dirty_link_transforms_ = robot_model_->getRootJoint();
}

void RobotState::update(bool force)
{
  // make sure we do everything from scratch if needed
  if (force) {
    std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
    dirty_link_transforms_ = robot_model_->getRootJoint();
  }

  // this actually triggers all needed updates
  updateCollisionBodyTransforms();
}

void RobotState::updateCollisionBodyTransforms()
{
  if (dirty_link_transforms_ != nullptr) {
    updateLinkTransforms();
  }

  if (dirty_collision_body_transforms_ != nullptr) {
    const std::vector<const LinkModel*>& links = dirty_collision_body_transforms_->getDescendantLinkModels();
    dirty_collision_body_transforms_ = nullptr;

    for (const LinkModel* link : links) {
      const EigenSTL::vector_Isometry3d& origin_transforms = link->getCollisionOriginTransforms();
      const std::vector<int>& origin_transforms_id = link->areCollisionOriginTransformsIdentity();
      const int index_co = link->getFirstCollisionBodyTransformIndex();
      const int index_l = link->getLinkIndex();
      for (size_t j = 0, end = origin_transforms.size(); j != end; ++j) {
        if (origin_transforms_id[j]) {
          global_collision_body_transforms_[index_co + j] = global_link_transforms_[index_l];
        }
        else {
          global_collision_body_transforms_[index_co + j].affine().noalias() =
            global_link_transforms_[index_l].affine() * origin_transforms[j].matrix();
        }
      }
    }
  }
}

void RobotState::updateLinkTransforms()
{
  if (dirty_link_transforms_ != nullptr) {
    updateLinkTransformsInternal(dirty_link_transforms_);
    if (dirty_collision_body_transforms_) {
      dirty_collision_body_transforms_ =
        robot_model_->getCommonRoot(dirty_collision_body_transforms_, dirty_link_transforms_);
    }
    else {
      dirty_collision_body_transforms_ = dirty_link_transforms_;
    }
    dirty_link_transforms_ = nullptr;
  }
}

void RobotState::updateLinkTransformsInternal(const JointModel* start)
{
  for (const LinkModel* link : start->getDescendantLinkModels()) {
    int idx_link = link->getLinkIndex();
    const LinkModel* parent = link->getParentLinkModel();
    if (parent)  // root JointModel will not have a parent
    {
      int idx_parent = parent->getLinkIndex();
      if (link->parentJointIsFixed()) {  // fixed joint
        global_link_transforms_[idx_link].affine().noalias() =
          global_link_transforms_[idx_parent].affine() * link->getJointOriginTransform().matrix();
      }
      else  // non-fixed joint
      {
        if (link->jointOriginTransformIsIdentity()) {  // Link has identity transform
          global_link_transforms_[idx_link].affine().noalias() =
            global_link_transforms_[idx_parent].affine() * getJointTransform(link->getParentJointModel()).matrix();
        }
        else {  // Link has non-identity transform
          global_link_transforms_[idx_link].affine().noalias() =
            global_link_transforms_[idx_parent].affine() * link->getJointOriginTransform().matrix() *
            getJointTransform(link->getParentJointModel()).matrix();
        }
      }
    }
    else  // is the origin / root / 'model frame'
    {
      const JointModel* root_joint = link->getParentJointModel();
      if (root_joint->getVariableCount() == 0) {
        // The root joint doesn't have any variables: avoid calling getJointTransform() on it.
        global_link_transforms_[idx_link] = Eigen::Isometry3d::Identity();
      }
      else if (link->jointOriginTransformIsIdentity()) {
        global_link_transforms_[idx_link] = getJointTransform(root_joint);
      }
      else {
        global_link_transforms_[idx_link].affine().noalias() =
          link->getJointOriginTransform().affine() * getJointTransform(root_joint).matrix();
      }
    }
  }
}

const Eigen::Isometry3d& RobotState::getJointTransform(const JointModel* joint)
{
  const int idx = joint->getJointIndex();
  if (joint->getVariableCount() == 0) {
    return variable_joint_transforms_[idx];
  }

  uint8_t& dirty = dirty_joint_transforms_[idx];
  if (dirty) {
    joint->computeTransform(&position_.at(joint->getFirstVariableIndex()), variable_joint_transforms_[idx]);
    dirty = 0;
  }
  return variable_joint_transforms_[idx];
}

double RobotState::distance(const RobotState& other, const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return 0.;
  }
  const int idx = joint->getFirstVariableIndex();
  return joint->distance(&position_.at(idx), &other.position_.at(idx));
}

const Eigen::Isometry3d& RobotState::getFrameTransform(const std::string& frame_id, bool* frame_found)
{
  updateLinkTransforms();
  return static_cast<const RobotState*>(this)->getFrameTransform(frame_id, frame_found);
}

const Eigen::Isometry3d& RobotState::getFrameTransform(const std::string& frame_id, bool* frame_found) const
{
  const LinkModel* ignored_link;
  bool found;
  const auto& result = getFrameInfo(frame_id, ignored_link, found);

  if (frame_found) {
    *frame_found = found;
  }
  else if (!found) {
    RCLCPP_WARN(getLogger(), "getFrameTransform() did not find a frame with name %s.", frame_id.c_str());
  }

  return result;
}

const Eigen::Isometry3d&
RobotState::getFrameInfo(const std::string& frame_id, const LinkModel*& robot_link, bool& frame_found) const
{
  if (!frame_id.empty() && frame_id[0] == '/') {
    return getFrameInfo(frame_id.substr(1), robot_link, frame_found);
  }

  static const Eigen::Isometry3d IDENTITY_TRANSFORM = Eigen::Isometry3d::Identity();
  if (frame_id == robot_model_->getModelFrame()) {
    robot_link = robot_model_->getRootLink();
    frame_found = true;
    return IDENTITY_TRANSFORM;
  }
  if ((robot_link = robot_model_->getLinkModel(frame_id, &frame_found))) {
    assert(checkLinkTransforms());
    return global_link_transforms_[robot_link->getLinkIndex()];
  }
  robot_link = nullptr;

  robot_link = nullptr;
  frame_found = false;
  return IDENTITY_TRANSFORM;
}

bool RobotState::knowsFrameTransform(const std::string& frame_id) const
{
  if (!frame_id.empty() && frame_id[0] == '/') {
    return knowsFrameTransform(frame_id.substr(1));
  }
  if (robot_model_->hasLinkModel(frame_id)) {
    return true;
  }

  return false;
}

void RobotState::getRobotMarkers(
  visualization_msgs::msg::MarkerArray& arr,
  const std::vector<std::string>& link_names,
  const std_msgs::msg::ColorRGBA& color,
  const std::string& ns,
  const rclcpp::Duration& dur) const
{
  size_t cur_num = arr.markers.size();
  getRobotMarkers(arr, link_names);
  uint32_t id = cur_num;
  for (size_t i = cur_num; i < arr.markers.size(); ++i, ++id) {
    arr.markers[i].ns = ns;
    arr.markers[i].id = id;
    arr.markers[i].lifetime = dur;
    arr.markers[i].color = color;
  }
}

void RobotState::getRobotMarkers(visualization_msgs::msg::MarkerArray& arr, const std::vector<std::string>& link_names)
  const
{
  rclcpp::Clock clock;
  for (const std::string& link_name : link_names) {
    RCLCPP_DEBUG(getLogger(), "Trying to get marker for link '%s'", link_name.c_str());
    const LinkModel* link_model = robot_model_->getLinkModel(link_name);
    if (!link_model) {
      continue;
    }

    if (link_model->getShapes().empty()) {
      continue;
    }

    for (size_t j = 0; j < link_model->getShapes().size(); ++j) {
      visualization_msgs::msg::Marker mark;
      mark.header.frame_id = robot_model_->getModelFrame();
      mark.header.stamp = clock.now();

      // we prefer using the visual mesh, if a mesh is available and we have one body to render
      const std::string& mesh_resource = link_model->getVisualMeshFilename();
      if (mesh_resource.empty() || link_model->getShapes().size() > 1) {
        if (!shapes::constructMarkerFromShape(link_model->getShapes()[j].get(), mark)) {
          continue;
        }
        // if the object is invisible (0 volume) we skip it
        if (std::abs(mark.scale.x * mark.scale.y * mark.scale.z) < std::numeric_limits<double>::epsilon()) {
          continue;
        }
        mark.pose =
          tf2::toMsg(global_collision_body_transforms_[link_model->getFirstCollisionBodyTransformIndex() + j]);
      }
      else {
        mark.type = mark.MESH_RESOURCE;
        mark.mesh_use_embedded_materials = false;
        mark.mesh_resource = mesh_resource;
        const Eigen::Vector3d& mesh_scale = link_model->getVisualMeshScale();

        mark.scale.x = mesh_scale[0];
        mark.scale.y = mesh_scale[1];
        mark.scale.z = mesh_scale[2];
        mark.pose = tf2::toMsg(global_link_transforms_[link_model->getLinkIndex()] * link_model->getVisualMeshOrigin());
      }

      arr.markers.push_back(mark);
    }
  }
}

void RobotState::printStatePositions(std::ostream& out) const
{
  const std::vector<std::string>& nm = robot_model_->getVariableNames();
  for (size_t i = 0; i < nm.size(); ++i) {
    out << nm[i] << '=' << position_[i] << '\n';
  }
}

void RobotState::printDirtyInfo(std::ostream& out) const
{
  out << "  * Dirty Joint Transforms: \n";
  const std::vector<const JointModel*>& jm = robot_model_->getJointModels();
  for (const JointModel* joint : jm) {
    if (joint->getVariableCount() > 0 && dirtyJointTransform(joint)) {
      out << "    " << joint->getName() << '\n';
    }
  }
  out << "  * Dirty Link Transforms: " << (dirty_link_transforms_ ? dirty_link_transforms_->getName() : "NULL") << '\n';
  out << "  * Dirty Collision Body Transforms: "
      << (dirty_collision_body_transforms_ ? dirty_collision_body_transforms_->getName() : "NULL\n");
}

void RobotState::printStateInfo(std::ostream& out) const
{
  out << "Robot State @" << this << '\n';

  size_t n = robot_model_->getVariableCount();
  if (!position_.empty()) {
    out << "  * Position: ";
    for (size_t i = 0; i < n; ++i) {
      out << position_[i] << ' ';
    }
    out << '\n';
  }
  else {
    out << "  * Position: NULL\n";
  }

  if (!velocity_.empty()) {
    out << "  * Velocity: ";
    for (size_t i = 0; i < n; ++i) {
      out << velocity_[i] << ' ';
    }
    out << '\n';
  }
  else {
    out << "  * Velocity: NULL\n";
  }

  if (has_acceleration_) {
    out << "  * Acceleration: ";
    for (size_t i = 0; i < n; ++i) {
      out << effort_or_acceleration_[i] << ' ';
    }
    out << '\n';
  }
  else {
    out << "  * Acceleration: NULL\n";
  }

  out << "  * Dirty Link Transforms: " << (dirty_link_transforms_ ? dirty_link_transforms_->getName() : "NULL\n");
  out << "  * Dirty Collision Body Transforms: "
      << (dirty_collision_body_transforms_ ? dirty_collision_body_transforms_->getName() : "NULL\n");

  printTransforms(out);
}

void RobotState::printTransform(const Eigen::Isometry3d& transform, std::ostream& out) const
{
  if (checkIsometry(transform, CHECK_ISOMETRY_PRECISION, false)) {
    Eigen::Quaterniond q(transform.linear());
    out << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", "
        << transform.translation().z() << "], Q.xyzw = [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()
        << ']';
  }
  else {
    out << "[NON-ISOMETRY] "
        << transform.matrix().format(
             Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]"));
  }
  out << '\n';
}

void RobotState::printTransforms(std::ostream& out) const
{
  if (variable_joint_transforms_.empty()) {
    out << "No transforms computed\n";
    return;
  }

  out << "Joint transforms:\n";
  const std::vector<const JointModel*>& jm = robot_model_->getJointModels();
  for (const JointModel* joint : jm) {
    out << "  " << joint->getName();
    const int idx = joint->getJointIndex();
    if (dirty_joint_transforms_[idx]) {
      out << " [dirty]";
    }
    out << ": ";
    printTransform(variable_joint_transforms_[idx], out);
  }

  out << "Link poses:\n";
  const std::vector<const LinkModel*>& link_model = robot_model_->getLinkModels();
  for (const LinkModel* link : link_model) {
    out << "  " << link->getName() << ": ";
    printTransform(global_link_transforms_[link->getLinkIndex()], out);
  }
}

std::string RobotState::getStateTreeString() const
{
  std::stringstream ss;
  ss << "ROBOT: " << robot_model_->getName() << '\n';
  getStateTreeJointString(ss, robot_model_->getRootJoint(), "   ", true);
  return ss.str();
}

namespace
{
void getPoseString(std::ostream& ss, const Eigen::Isometry3d& pose, const std::string& pfx)
{
  ss.precision(3);
  for (int y = 0; y < 4; ++y) {
    ss << pfx;
    for (int x = 0; x < 4; ++x) {
      ss << std::setw(8) << pose(y, x) << ' ';
    }
    ss << '\n';
  }
}
}  // namespace

void RobotState::getStateTreeJointString(std::ostream& ss, const JointModel* jm, const std::string& pfx0, bool last) const
{
  std::string pfx = pfx0 + "+--";

  ss << pfx << "Joint: " << jm->getName() << '\n';

  pfx = pfx0 + (last ? "   " : "|  ");

  for (size_t i = 0; i < jm->getVariableCount(); ++i) {
    ss.precision(3);
    ss << pfx << jm->getVariableNames()[i] << std::setw(12) << position_[jm->getFirstVariableIndex() + i] << '\n';
  }

  const LinkModel* link_model = jm->getChildLinkModel();

  ss << pfx << "Link: " << link_model->getName() << '\n';
  getPoseString(ss, link_model->getJointOriginTransform(), pfx + "joint_origin:");
  if (!variable_joint_transforms_.empty()) {
    getPoseString(ss, variable_joint_transforms_[jm->getJointIndex()], pfx + "joint_variable:");
    getPoseString(ss, global_link_transforms_[link_model->getLinkIndex()], pfx + "link_global:");
  }

  for (std::vector<const JointModel*>::const_iterator it = link_model->getChildJointModels().begin();
       it != link_model->getChildJointModels().end();
       ++it) {
    getStateTreeJointString(ss, *it, pfx, it + 1 == link_model->getChildJointModels().end());
  }
}

std::ostream& operator<<(std::ostream& out, const RobotState& s)
{
  s.printStateInfo(out);
  return out;
}
}  // namespace tobas
