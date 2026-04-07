// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <geometric_shapes/check_isometry.h>
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

/* A link from the robot. Contains the constant transform applied to the link and its geometry */
class LinkModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct a link model named \e name
   *
   * @param name        The name of the link
   * @param link_index  The link index in the RobotModel
   */
  LinkModel(const std::string& name, size_t link_index);
  ~LinkModel();

  /* The name of this link */
  const std::string& getName() const
  {
    return name_;
  }

  /* The index of this joint when traversing the kinematic tree in depth first fashion */
  size_t getLinkIndex() const
  {
    return link_index_;
  }

  int getFirstCollisionBodyTransformIndex() const
  {
    return first_collision_body_transform_index_;
  }

  void setFirstCollisionBodyTransformIndex(int index)
  {
    first_collision_body_transform_index_ = index;
  }

  /* Get the joint model whose child this link is. There will always be a parent joint */
  const JointModel* getParentJointModel() const
  {
    return parent_joint_model_;
  }

  void setParentJointModel(const JointModel* joint);

  /* Get the link model whose child this link is (through some joint). There may not always be a parent link
   * (nullptr is returned for the root link) */
  const LinkModel* getParentLinkModel() const
  {
    return parent_link_model_;
  }

  void setParentLinkModel(const LinkModel* link)
  {
    parent_link_model_ = link;
  }

  /* A link may have 0 or more child joints. From those joints there will certainly be other descendant links */
  const std::vector<const JointModel*>& getChildJointModels() const
  {
    return child_joint_models_;
  }

  void addChildJointModel(const JointModel* joint)
  {
    child_joint_models_.push_back(joint);
  }

  /* When transforms are computed for this link,
      they are usually applied to the link's origin. The
      joint origin transform acts as an offset -- it is
      pre-applied before any other transform. The
      transform is guaranteed to be a valid isometry. */
  const Eigen::Isometry3d& getJointOriginTransform() const
  {
    return joint_origin_transform_;
  }

  bool jointOriginTransformIsIdentity() const
  {
    return joint_origin_transform_is_identity_;
  }

  bool parentJointIsFixed() const
  {
    return is_parent_joint_fixed_;
  }

  void setJointOriginTransform(const Eigen::Isometry3d& transform);

  /* In addition to the link transform, the geometry
      of a link that is used for collision checking may have
      a different offset itself, with respect to the origin.
      The transform is guaranteed to be a valid isometry. */
  const EigenSTL::vector_Isometry3d& getCollisionOriginTransforms() const
  {
    return collision_origin_transform_;
  }

  /* Return flags for each transform specifying whether they are identity or not */
  const std::vector<int>& areCollisionOriginTransformsIdentity() const
  {
    return collision_origin_transform_is_identity_;
  }

  /* Get shape associated to the collision geometry for this link */
  const std::vector<shapes::ShapeConstPtr>& getShapes() const
  {
    return shapes_;
  }

  void setGeometry(const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& origins);

  /* Remember that \e link_model is attached to this link using a fixed transform */
  void addAssociatedFixedTransform(const LinkModel* link_model, const Eigen::Isometry3d& transform)
  {
    ASSERT_ISOMETRY(transform);  // unsanitized input, could contain a non-isometry
    associated_fixed_transforms_[link_model] = transform;
  }

  void setVisualMesh(const std::string& visual_mesh, const Eigen::Isometry3d& origin, const Eigen::Vector3d& scale);

private:
  /* Name of the link */
  std::string name_;

  /* Index of the transform for this link in the full robot frame */
  size_t link_index_;

  /* JointModel that connects this link to the parent link */
  const JointModel* parent_joint_model_;

  /* The parent link model (nullptr for the root link) */
  const LinkModel* parent_link_model_;

  /* List of directly descending joints (each connects to a child link) */
  std::vector<const JointModel*> child_joint_models_;

  /* True if the parent joint of this link is fixed */
  bool is_parent_joint_fixed_;

  /* True of the joint origin transform is identity */
  bool joint_origin_transform_is_identity_;

  /* The constant transform applied to the link (local) */
  Eigen::Isometry3d joint_origin_transform_;

  /* The constant transform applied to the collision geometry of the link (local) */
  EigenSTL::vector_Isometry3d collision_origin_transform_;

  /* Flag indicating if the constant transform applied to the collision geometry of the link (local) is
   * identity; use int instead of bool to avoid bit operations */
  std::vector<int> collision_origin_transform_is_identity_;

  /* The set of links that are attached to this one via fixed transforms */
  LinkTransformMap associated_fixed_transforms_;

  /* The collision geometry of the link */
  std::vector<shapes::ShapeConstPtr> shapes_;

  /* Filename associated with the visual geometry mesh of this link. If empty, no mesh was used. */
  std::string visual_mesh_filename_;

  /* The additional origin transform for the mesh */
  Eigen::Isometry3d visual_mesh_origin_;

  /* Scale factor associated with the visual geometry mesh of this link. */
  Eigen::Vector3d visual_mesh_scale_;

  /* Index of the transform for the first shape that makes up the geometry of this link in the full robot state
   */
  int first_collision_body_transform_index_;
};
}  // namespace tobas
