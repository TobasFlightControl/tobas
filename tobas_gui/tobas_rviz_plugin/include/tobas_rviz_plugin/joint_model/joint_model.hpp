// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>

#include <tobas_visualization_msgs/msg/joint_limits.hpp>

namespace tobas
{
class LinkModel;
class JointModel;

/* Data type for holding mappings from variable names to their position in a state vector */
using VariableIndexMap = std::map<std::string, size_t>;

/* Map of names to instances for JointModel */
using JointModelMap = std::map<std::string, JointModel*>;

/**
 * @brief A joint from the robot.
 * Models the transform that this joint applies in the kinematic chain.
 * A joint consists of multiple variables.
 * In the simplest case, when the joint is a single DOF,
 * there is only one variable and its name is the same as the joint's name.
 * For multi-DOF joints, each variable has a local name (e.g., \e x, \e y)
 * but the full variable name as seen from the outside of this class is a concatenation of the "joint name"/"local name"
 * (e.g., a joint named 'base' with local variables 'x' and 'y' will store its full variable names as 'base/x' and
 * 'base/y'). Local names are never used to reference variables directly.
 */
class JointModel
{
public:
  /* The different types of joints we support */
  enum JointType
  {
    UNKNOWN,
    REVOLUTE,
    PRISMATIC,
    PLANAR,
    FLOATING,
    FIXED,
  };

  /**
   * @brief Constructs a joint named \e name
   *
   * @param name                   The joint name
   * @param index                  The index of the joint in the RobotModel
   * @param first_variable_index   The index of the first variable in the RobotModel
   */
  explicit JointModel(const std::string& name, size_t joint_index, size_t first_variable_index);
  virtual ~JointModel();

  /* Get the name of the joint */
  const std::string& getName() const
  {
    return name_;
  }

  /* Get the type of joint */
  JointType getType() const
  {
    return type_;
  }

  /**
   * @brief Get the link that this joint connects to.
   * The robot is assumed to start with a joint,
   * so the root joint will return nullptr here.
   */
  const LinkModel* getParentLinkModel() const
  {
    return parent_link_model_;
  }

  /* Get the link that this joint connects to. There will always be such a link */
  const LinkModel* getChildLinkModel() const
  {
    return child_link_model_;
  }

  void setParentLinkModel(const LinkModel* link)
  {
    parent_link_model_ = link;
  }

  void setChildLinkModel(const LinkModel* link)
  {
    child_link_model_ = link;
  }

  /**
   * @brief Get the names of the variables that make up this joint, in the order they appear in corresponding states.
   * For single DOF joints, this will be just the joint name. For multi-DOF joints these will be the joint name
   * followed by "/", followed by the local names of the variables
   */
  const std::vector<std::string>& getVariableNames() const
  {
    return variable_names_;
  }

  /* Get the number of variables that describe this joint */
  size_t getVariableCount() const
  {
    return variable_names_.size();
  }

  /* Get the index of this joint's first variable within the full robot state */
  size_t getFirstVariableIndex() const
  {
    return first_variable_index_;
  }

  /* Get the index of this joint within the robot model */
  size_t getJointIndex() const
  {
    return joint_index_;
  }

  /* Get the index of the variable within this joint */
  size_t getLocalVariableIndex(const std::string& variable) const;

  /**
   * @brief Provide a default value for the joint given the joint variable bounds.
   * Most joints will use the default implementation provided in this base class,
   * but the quaternion for example needs a different implementation.
   * Enough memory is assumed to be allocated.
   */
  virtual void getVariableDefaultPositions(double* values) const = 0;

  /* Get the joint this one is mimicking */
  const JointModel* getMimic() const
  {
    return mimic_;
  }

  /* If mimicking a joint, this is the offset added to that joint's value */
  double getMimicOffset() const
  {
    return mimic_offset_;
  }

  /* If mimicking a joint, this is the multiplicative factor for that joint's value */
  double getMimicFactor() const
  {
    return mimic_factor_;
  }

  /* Mark this joint as mimicking \e mimic using \e factor and \e offset */
  void setMimic(const JointModel* mimic, double factor, double offset);

  /* The joint models whose values would be modified if the value of this joint changed */
  const std::vector<const JointModel*>& getMimicRequests() const
  {
    return mimic_requests_;
  }

  /* Notify this joint that there is another joint that mimics it */
  void addMimicRequest(const JointModel* joint);
  void addDescendantJointModel(const JointModel* joint);
  void addDescendantLinkModel(const LinkModel* link);

  /* Get all the link models that descend from this joint, in the kinematic tree */
  const std::vector<const LinkModel*>& getDescendantLinkModels() const
  {
    return descendant_link_models_;
  }

  /* Get all the joint models that descend from this joint, in the kinematic tree */
  const std::vector<const JointModel*>& getDescendantJointModels() const
  {
    return descendant_joint_models_;
  }

  /* Get all the non-fixed joint models that descend from this joint, in the kinematic tree */
  const std::vector<const JointModel*>& getNonFixedDescendantJointModels() const
  {
    return non_fixed_descendant_joint_models_;
  }

  /**
   * @brief Given the joint values for a joint, compute the corresponding transform.
   * The computed transform is guaranteed to be a valid isometry.
   */
  virtual void computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const = 0;

  /**
   * @brief Given the transform generated by joint, compute the corresponding joint values.
   * Make sure the passed transform is a valid isometry.
   */
  virtual void computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const = 0;

protected:
  /* The type of joint */
  JointType type_;

  /* The local names to use for the variables that make up this joint */
  std::vector<std::string> local_variable_names_;

  /* The full names to use for the variables that make up this joint */
  std::vector<std::string> variable_names_;

  /* Map from variable names to the corresponding index in variable_names_
   * (indexing makes sense within the JointModel only) */
  VariableIndexMap variable_index_map_;

  /* The link before this joint */
  const LinkModel* parent_link_model_;

  /* The link after this joint */
  const LinkModel* child_link_model_;

  /* The joint this one mimics (nullptr for joints that do not mimic) */
  const JointModel* mimic_;

  /* The multiplier to the mimic joint */
  double mimic_factor_;

  /* The offset to the mimic joint */
  double mimic_offset_;

  /* The set of joints that should get a value copied to them when this joint changes */
  std::vector<const JointModel*> mimic_requests_;

  /* Pointers to all the links that will be moved if this joint changes value */
  std::vector<const LinkModel*> descendant_link_models_;

  /* Pointers to all the joints that follow this one in the kinematic tree (including mimic joints) */
  std::vector<const JointModel*> descendant_joint_models_;

  /* Pointers to all the joints that follow this one in the kinematic tree,
   * including mimic joints, but excluding fixed joints */
  std::vector<const JointModel*> non_fixed_descendant_joint_models_;

private:
  /* Name of the joint */
  const std::string name_;

  /* Index for this joint in the array of joints of the complete model */
  const size_t joint_index_;

  /* The index of this joint's first variable, in the complete robot state */
  const size_t first_variable_index_;
};
}  // namespace tobas
