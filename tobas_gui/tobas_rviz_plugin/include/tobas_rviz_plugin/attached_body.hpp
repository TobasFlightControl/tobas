#pragma once

#include <geometric_shapes/check_isometry.h>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <set>
#include <functional>

#include "./link_model.hpp"
#include "./transforms.hpp"

namespace tobas
{
class AttachedBody;
typedef std::function<void(AttachedBody* body, bool attached)> AttachedBodyCallback;

/** @brief Object defining bodies that can be attached to robot links.
 *
 * This is useful when handling objects picked up by the robot. */
class AttachedBody
{
public:
  /** \brief Construct an attached body for a specified \e link.
   *
   * The name of this body is \e id and it consists of \e shapes that attach to the link by the transforms
   * \e shape_poses. The set of links that are allowed to be touched by this object is specified by \e touch_links.
   * detach_posture may describe a detach motion for the gripper when placing the object.
   * The shape and subframe poses are relative to the \e pose, and \e pose is relative to the parent link. */
  AttachedBody(
    const LinkModel* parent,
    const std::string& id,
    const Eigen::Isometry3d& pose,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const EigenSTL::vector_Isometry3d& shape_poses,
    const std::set<std::string>& touch_links,
    const trajectory_msgs::msg::JointTrajectory& detach_posture,
    const FixedTransformsMap& subframe_poses = FixedTransformsMap());

  ~AttachedBody();

  /** \brief Get the name of the attached body */
  const std::string& getName() const
  {
    return id_;
  }

  /** \brief Get the pose of the attached body relative to the parent link */
  const Eigen::Isometry3d& getPose() const
  {
    return pose_;
  }

  /** \brief Get the pose of the attached body, relative to the world */
  const Eigen::Isometry3d& getGlobalPose() const
  {
    return global_pose_;
  }

  /** \brief Get the name of the link this body is attached to */
  const std::string& getAttachedLinkName() const
  {
    return parent_link_model_->getName();
  }

  /** \brief Get the model of the link this body is attached to */
  const LinkModel* getAttachedLink() const
  {
    return parent_link_model_;
  }

  /** \brief Get the shapes that make up this attached body */
  const std::vector<shapes::ShapeConstPtr>& getShapes() const
  {
    return shapes_;
  }

  /** \brief Get the shape poses (the transforms to the shapes of this body, relative to the pose). The returned
   *  transforms are guaranteed to be valid isometries. */
  const EigenSTL::vector_Isometry3d& getShapePoses() const
  {
    return shape_poses_;
  }

  /** \brief Get the links that the attached body is allowed to touch */
  const std::set<std::string>& getTouchLinks() const
  {
    return touch_links_;
  }

  /** \brief Return the posture that is necessary for the object to be released, (if any). This is useful for example
     when storing the configuration of a gripper holding an object */
  const trajectory_msgs::msg::JointTrajectory& getDetachPosture() const
  {
    return detach_posture_;
  }

  /** \brief Get the fixed transforms (the transforms to the shapes of this body, relative to the link). The returned
   *  transforms are guaranteed to be valid isometries. */
  const EigenSTL::vector_Isometry3d& getShapePosesInLinkFrame() const
  {
    return shape_poses_in_link_frame_;
  }

  /** \brief Get subframes of this object (relative to the object pose). The returned transforms are guaranteed to be
   * valid isometries. */
  const FixedTransformsMap& getSubframes() const
  {
    return subframe_poses_;
  }

  /** \brief Get subframes of this object (in the world frame) */
  const FixedTransformsMap& getGlobalSubframeTransforms() const
  {
    return global_subframe_poses_;
  }

  /** \brief Set all subframes of this object.
   *
   * Use these to define points of interest on the object to plan with
   * (e.g. screwdriver/tip, kettle/spout, mug/base).
   */
  void setSubframeTransforms(const FixedTransformsMap& subframe_poses)
  {
    for (const auto& t : subframe_poses)
    {
      ASSERT_ISOMETRY(t.second)  // unsanitized input, could contain a non-isometry
    }
    subframe_poses_ = subframe_poses;
  }

  /** \brief Get the fixed transform to a named subframe on this body (relative to the body's pose)
   *
   * The frame_name needs to have the object's name prepended (e.g. "screwdriver/tip" returns true if the object's
   * name is "screwdriver"). Returns an identity transform if frame_name is unknown (and set found to false).
   * The returned transform is guaranteed to be a valid isometry. */
  const Eigen::Isometry3d& getSubframeTransform(const std::string& frame_name, bool* found = nullptr) const;

  /** \brief Get the fixed transform to a named subframe on this body, relative to the world frame.
   * The frame_name needs to have the object's name prepended (e.g. "screwdriver/tip" returns true if the object's
   * name is "screwdriver"). Returns an identity transform if frame_name is unknown (and set found to false).
   * The returned transform is guaranteed to be a valid isometry. */
  const Eigen::Isometry3d& getGlobalSubframeTransform(const std::string& frame_name, bool* found = nullptr) const;

  /** \brief Check whether a subframe of given @frame_name is present in this object.
   *
   * The frame_name needs to have the object's name prepended (e.g. "screwdriver/tip" returns true if the object's
   * name is "screwdriver"). */
  bool hasSubframeTransform(const std::string& frame_name) const;

  /** \brief Get the global transforms (in world frame) for the collision bodies. The returned transforms are
   *  guaranteed to be valid isometries. */
  const EigenSTL::vector_Isometry3d& getGlobalCollisionBodyTransforms() const
  {
    return global_collision_body_transforms_;
  }

  /** \brief Set the padding for the shapes of this attached object */
  void setPadding(double padding);

  /** \brief Set the scale for the shapes of this attached object */
  void setScale(double scale);

  /** \brief Recompute global_collision_body_transform given the transform of the parent link */
  void computeTransform(const Eigen::Isometry3d& parent_link_global_transform);

private:
  /** \brief The link that owns this attached body */
  const LinkModel* parent_link_model_;

  /** \brief string id for reference */
  std::string id_;

  /** \brief The transform from the parent link to the attached body's pose*/
  Eigen::Isometry3d pose_;

  /** \brief The transform from the model frame to the attached body's pose  */
  Eigen::Isometry3d global_pose_;

  /** \brief The geometries of the attached body */
  std::vector<shapes::ShapeConstPtr> shapes_;

  /** \brief The transforms from the object's pose to the object's geometries*/
  EigenSTL::vector_Isometry3d shape_poses_;

  /** \brief The transforms from the link to the object's geometries*/
  EigenSTL::vector_Isometry3d shape_poses_in_link_frame_;

  /** \brief The global transforms for the attached bodies (computed by forward kinematics) */
  EigenSTL::vector_Isometry3d global_collision_body_transforms_;

  /** \brief The set of links this body is allowed to touch */
  std::set<std::string> touch_links_;

  /** \brief Posture of links for releasing the object (if any). This is useful for example when storing
      the configuration of a gripper holding an object */
  trajectory_msgs::msg::JointTrajectory detach_posture_;

  /** \brief Transforms to subframes on the object, relative to the object's pose. */
  FixedTransformsMap subframe_poses_;

  /** \brief Transforms to subframes on the object, relative to the model frame. */
  FixedTransformsMap global_subframe_poses_;
};
}  // namespace tobas
