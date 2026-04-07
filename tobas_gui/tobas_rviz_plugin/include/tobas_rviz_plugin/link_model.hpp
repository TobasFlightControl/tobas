// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>
#include <string>
#include <vector>

#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <eigen3/Eigen/Geometry>

#include "./shape.hpp"

namespace tobas
{
class JointModel;
class LinkModel;

/* Map from link model instances to Eigen transforms. */
using LinkTransformMap = std::map<
  const LinkModel*,
  Eigen::Isometry3d,
  std::less<const LinkModel*>,
  Eigen::aligned_allocator<std::pair<const LinkModel* const, Eigen::Isometry3d>>>;

/* A link from the robot. Contains the constant transform applied to the link and its geometry. */
class LinkModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit LinkModel(const std::string& name, size_t link_index);
  ~LinkModel();

  /* The name of this link. */
  const std::string& getName() const;

  /* The index of this joint when traversing the kinematic tree in depth first fashion. */
  size_t getLinkIndex() const;

  int getFirstCollisionBodyTransformIndex() const;

  void setFirstCollisionBodyTransformIndex(int index);

  /* Get the joint model whose child this link is. There will always be a parent joint. */
  const JointModel* getParentJointModel() const;

  void setParentJointModel(const JointModel* joint);

  /* Get the link model whose child this link is (through some joint). There may not always be a parent link. */
  const LinkModel* getParentLinkModel() const;

  void setParentLinkModel(const LinkModel* link);

  /* A link may have 0 or more child joints. From those joints there will certainly be other descendant links. */
  const std::vector<const JointModel*>& getChildJointModels() const;

  void addChildJointModel(const JointModel* joint);

  const Eigen::Isometry3d& getJointOriginTransform() const;

  bool jointOriginTransformIsIdentity() const;

  bool parentJointIsFixed() const;

  void setJointOriginTransform(const Eigen::Isometry3d& transform);

  const EigenSTL::vector_Isometry3d& getCollisionOriginTransforms() const;

  /* Return flags for each transform specifying whether they are identity or not. */
  const std::vector<int>& areCollisionOriginTransformsIdentity() const;

  /* Get shape associated to the collision geometry for this link. */
  const std::vector<shapes::ShapeConstPtr>& getShapes() const;

  void setGeometry(const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& origins);

  /* Remember that \e link_model is attached to this link using a fixed transform. */
  void addAssociatedFixedTransform(const LinkModel* link_model, const Eigen::Isometry3d& transform);

  void setVisualMesh(const std::string& visual_mesh, const Eigen::Isometry3d& origin, const Eigen::Vector3d& scale);

private:
  /* Name of the link */
  const std::string name_;

  /* Index of the transform for this link in the full robot frame */
  const size_t link_index_;

  /* JointModel that connects this link to the parent link */
  const JointModel* parent_joint_model_ = nullptr;

  /* The parent link model (nullptr for the root link) */
  const LinkModel* parent_link_model_ = nullptr;

  /* List of directly descending joints (each connects to a child link) */
  std::vector<const JointModel*> child_joint_models_;

  /* True if the parent joint of this link is fixed */
  bool is_parent_joint_fixed_ = false;

  /* True of the joint origin transform is identity */
  bool joint_origin_transform_is_identity_ = true;

  /* The constant transform applied to the link (local) */
  Eigen::Isometry3d joint_origin_transform_;

  /* The constant transform applied to the collision geometry of the link (local) */
  EigenSTL::vector_Isometry3d collision_origin_transform_;

  /* Flag indicating if the constant transform applied to the collision geometry of the link (local) is identity */
  std::vector<int> collision_origin_transform_is_identity_;

  /* The set of links that are attached to this one via fixed transforms */
  LinkTransformMap associated_fixed_transforms_;

  /* The collision geometry of the link */
  std::vector<shapes::ShapeConstPtr> shapes_;

  /* Filename associated with the visual geometry mesh of this link. If empty, no mesh was used. */
  std::string visual_mesh_filename_;

  /* The additional origin transform for the mesh */
  Eigen::Isometry3d visual_mesh_origin_;

  /* Scale factor associated with the visual geometry mesh of this link */
  Eigen::Vector3d visual_mesh_scale_;

  /* Index of the transform for the first shape that makes up the geometry of this link in the full robot state */
  int first_collision_body_transform_index_ = -1;
};
}  // namespace tobas
