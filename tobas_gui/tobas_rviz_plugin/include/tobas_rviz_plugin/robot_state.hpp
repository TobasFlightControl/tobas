// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <sensor_msgs/msg/joint_state.hpp>

#include "./robot_model.hpp"

namespace tobas
{
/* Representation of a robot's state. This includes position, velocity, acceleration and effort. */
class RobotState
{
public:
  using SharedPtr = std::shared_ptr<RobotState>;
  using ConstSharedPtr = std::shared_ptr<const RobotState>;

  explicit RobotState(const RobotModel::ConstSharedPtr& robot_model);

  const RobotModel::ConstSharedPtr& getRobotModel() const;

  /* Get a raw pointer to the positions of the variables stored in this state. */
  double* getVariablePositions();

  /* Get a raw pointer to the positions of the variables stored in this state. */
  const double* getVariablePositions() const;

  /* It is assumed \e positions is an array containing the new positions for all variables in this state.
   * Those values are copied into the state. */
  void setVariablePositions(const double* position);
  void setVariablePositions(const std::vector<double>& position);

  /* Set the positions of a set of variables. If unknown variable names are specified, an exception is thrown. */
  void setVariablePositions(const std::map<std::string, double>& variable_map);

  /* Set the positions of a set of variables. If unknown variable names are specified, an exception is thrown.
   * Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void
  setVariablePositions(const std::map<std::string, double>& variable_map, std::vector<std::string>& missing_variables);

  /* Set the positions of a set of variables. If unknown variable names are specified, an exception is thrown.
   * Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void
  setVariablePositions(const std::vector<std::string>& variable_names, const std::vector<double>& variable_position);

  /* Set the position of a single variable. An exception is thrown if the variable name is not known. */
  void setVariablePosition(const std::string& variable, double value);

  /* Set the position of a single variable.
   * The variable is specified by its index (a value associated by the RobotModel to each variable). */
  void setVariablePosition(int index, double value);

  /* Get the position of a particular variable. An exception is thrown if the variable is not known. */
  double getVariablePosition(const std::string& variable) const;

  /* Get the position of a particular variable.
   * The variable is specified by its index.
   * No checks are performed for the validity of the index passed. */
  double getVariablePosition(int index) const;

  /* By default, if velocities are never set or initialized,
   * the state remembers that there are no velocities set.
   * This is useful to know when serializing or copying the state. */
  bool hasVelocities() const;

  /* Get raw access to the velocities of the variables that make up this state.
   * The values are in the same order as reported by getVariableNames(). */
  double* getVariableVelocities();

  /* Get const access to the velocities of the variables that make up this state.
   * The values are in the same order as reported by getVariableNames(). */
  const double* getVariableVelocities() const;

  /* Given an array with velocity values for all variables, set those values as the velocities in this state. */
  void setVariableVelocities(const double* velocity);

  /* Given an array with velocity values for all variables, set those values as the velocities in this state. */
  void setVariableVelocities(const std::vector<double>& velocity);

  /* Set the velocities of a set of variables. If unknown variable names are specified, an exception is thrown. */
  void setVariableVelocities(const std::map<std::string, double>& variable_map);

  /* Set the velocities of a set of variables. If unknown variable names are specified, an exception is thrown.
   * Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void
  setVariableVelocities(const std::map<std::string, double>& variable_map, std::vector<std::string>& missing_variables);

  /* Set the velocities of a set of variables. If unknown variable names are specified, an exception is thrown. */
  void
  setVariableVelocities(const std::vector<std::string>& variable_names, const std::vector<double>& variable_velocity);

  /* Set the velocity of a variable. If an unknown variable name is specified, an exception is thrown. */
  void setVariableVelocity(const std::string& variable, double value);

  /* Set the velocity of a single variable.
   * The variable is specified by its index (a value associated by the RobotModel to each variable). */
  void setVariableVelocity(int index, double value);

  /* Get the velocity of a particular variable. An exception is thrown if the variable is not known. */
  double getVariableVelocity(const std::string& variable) const;

  /* Get the velocity of a particular variable.
   * The variable is specified by its index.
   * No checks are performed for the validity of the index passed. */
  double getVariableVelocity(int index) const;

  /* By default, if accelerations are never set or initialized, the state remembers that there are no accelerations set.
   * This is useful to know when serializing or copying the state.
   * If hasAccelerations() reports true, hasEffort() will certainly report false. */
  bool hasAccelerations() const;

  /* Get raw access to the accelerations of the variables that make up this state.
   * The values are in the same order as reported by getVariableNames().
   * The area of memory overlaps with effort (effort and acceleration should not be set at the same time). */
  double* getVariableAccelerations();

  /* Get const raw access to the accelerations of the variables that make up this state.
   * The values are in the same order as reported by getVariableNames(). */
  const double* getVariableAccelerations() const;

  /* Given an array with acceleration values for all variables, set those values as the accelerations in this state. */
  void setVariableAccelerations(const double* acceleration);

  /* Given an array with acceleration values for all variables, set those values as the accelerations in this state. */
  void setVariableAccelerations(const std::vector<double>& acceleration);

  /* Set the accelerations of a set of variables. If unknown variable names are specified, an exception is thrown. */
  void setVariableAccelerations(const std::map<std::string, double>& variable_map);

  /* Set the accelerations of a set of variables. If unknown variable names are specified, an exception is thrown.
   * Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void setVariableAccelerations(
    const std::map<std::string, double>& variable_map,
    std::vector<std::string>& missing_variables);

  /* Set the accelerations of a set of variables. If unknown variable names are specified, an exception is thrown. */
  void setVariableAccelerations(
    const std::vector<std::string>& variable_names,
    const std::vector<double>& variable_acceleration);

  /* Set the acceleration of a variable. If an unknown variable name is specified, an exception is thrown. */
  void setVariableAcceleration(const std::string& variable, double value);

  /* Set the acceleration of a single variable.
   * The variable is specified by its index (a value associated by the RobotModel to each variable). */
  void setVariableAcceleration(int index, double value);

  /* Get the acceleration of a particular variable. An exception is thrown if the variable is not known. */
  double getVariableAcceleration(const std::string& variable) const;

  /* Get the acceleration of a particular variable.
   * The variable is specified by its index.
   * No checks are performed for the validity of the index passed */
  double getVariableAcceleration(int index) const;

  /* By default, if effort is never set or initialized, the state remembers that there is no effort set.
   * This is useful to know when serializing or copying the state.
   * If hasEffort() reports true, hasAccelerations() will certainly report false. */
  bool hasEffort() const;

  /* Get raw access to the effort of the variables that make up this state.
   * The values are in the same order as reported by getVariableNames().
   * The area of memory overlaps with accelerations (effort and acceleration should not be set at the same time). */
  double* getVariableEffort();

  /* Get const raw access to the effort of the variables that make up this state.
   * The values are in the same order as reported by getVariableNames(). */
  const double* getVariableEffort() const;

  /* Given an array with effort values for all variables, set those values as the effort in this state. */
  void setVariableEffort(const double* effort);

  /* Given an array with effort values for all variables, set those values as the effort in this state. */
  void setVariableEffort(const std::vector<double>& effort);

  /* Set the effort of a set of variables. If unknown variable names are specified, an exception is thrown. */
  void setVariableEffort(const std::map<std::string, double>& variable_map);

  /* Set the effort of a set of variables. If unknown variable names are specified, an exception is thrown.
   * Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void setVariableEffort(const std::map<std::string, double>& variable_map, std::vector<std::string>& missing_variables);

  /* Set the effort of a set of variables. If unknown variable names are specified, an exception is thrown. */
  void
  setVariableEffort(const std::vector<std::string>& variable_names, const std::vector<double>& variable_acceleration);

  /* Set the effort of a variable. If an unknown variable name is specified, an exception is thrown. */
  void setVariableEffort(const std::string& variable, double value);

  /* Set the effort of a single variable.
   * The variable is specified by its index (a value associated by the RobotModel to each variable). */
  void setVariableEffort(int index, double value);

  /* Get the effort of a particular variable. An exception is thrown if the variable is not known. */
  double getVariableEffort(const std::string& variable) const;

  /* Get the effort of a particular variable.
   * The variable is specified by its index. No checks are performed for the validity of the index passed. */
  double getVariableEffort(int index) const;

  /* Getting and setting joint positions, velocities, accelerations and effort for a single joint.
   * The joint might be multi-DOF, i.e. require more than one variable to set.
   * See setVariablePositions(), setVariableVelocities(), setVariableEffort() to handle multiple joints. */
  void setJointPositions(const std::string& joint_name, const double* position);
  void setJointPositions(const std::string& joint_name, const std::vector<double>& position);
  void setJointPositions(const JointModel* joint, const std::vector<double>& position);
  void setJointPositions(const JointModel* joint, const double* position);
  void setJointPositions(const std::string& joint_name, const Eigen::Isometry3d& transform);
  void setJointPositions(const JointModel* joint, const Eigen::Isometry3d& transform);

  void setJointVelocities(const JointModel* joint, const double* velocity);

  void setJointEfforts(const JointModel* joint, const double* effort);

  const double* getJointPositions(const std::string& joint_name) const;
  const double* getJointPositions(const JointModel* joint) const;

  const double* getJointVelocities(const std::string& joint_name) const;
  const double* getJointVelocities(const JointModel* joint) const;

  const double* getJointAccelerations(const std::string& joint_name) const;
  const double* getJointAccelerations(const JointModel* joint) const;

  const double* getJointEffort(const std::string& joint_name) const;
  const double* getJointEffort(const JointModel* joint) const;

  void setVariableValues(const sensor_msgs::msg::JointState& msg);

  /* Set all joints to their default positions.
   * The default position is 0, or if that is not within bounds then half way between min and max bound. */
  void setToDefaultValues();

  /* Update the transforms for the collision bodies. This call is needed before calling collision checking.
   * If updating link transforms or joint transforms is needed, the corresponding updates are also triggered. */
  void updateCollisionBodyTransforms();

  /* Update the reference frame transforms for links.
  This call is needed before using the transforms of links for coordinate transforms. */
  void updateLinkTransforms();

  /* Update all transforms. */
  void update(bool force = false);

  /* Get the link transform wrt. the root link (model frame) of the RobotModel.
   * This is typically the root link of the URDF unless a virtual joint is present.
   * Checks the cache and if there are any dirty (non-updated) transforms, first updates them as needed.
   * A related, more comprehensive function is |getFrameTransform|,
   * which additionally to link frames also searches for attached object frames and their subframes.
   * This will throw an exception if the passed link is not found.
   * The returned transformation is always a valid isometry. */
  const Eigen::Isometry3d& getGlobalLinkTransform(const std::string& link_name);
  const Eigen::Isometry3d& getGlobalLinkTransform(const LinkModel* link);
  const Eigen::Isometry3d& getGlobalLinkTransform(const std::string& link_name) const;
  const Eigen::Isometry3d& getGlobalLinkTransform(const LinkModel* link) const;

  const Eigen::Isometry3d& getJointTransform(const std::string& joint_name);
  const Eigen::Isometry3d& getJointTransform(const JointModel* joint);
  const Eigen::Isometry3d& getJointTransform(const std::string& joint_name) const;
  const Eigen::Isometry3d& getJointTransform(const JointModel* joint) const;

  bool dirtyJointTransform(const JointModel* joint) const;

  bool dirtyLinkTransforms() const;

  bool dirtyCollisionBodyTransforms() const;

  /* Returns true if anything in this state is dirty. */
  bool dirty() const;

private:
  void init();

  void markDirtyJointTransforms(const JointModel* joint);

  void markVelocity();
  void markAcceleration();
  void markEffort();

  void updateMimicJoint(const JointModel* joint);

  void updateLinkTransformsInternal(const JointModel* start);

  void
  getMissingKeys(const std::map<std::string, double>& variable_map, std::vector<std::string>& missing_variables) const;

  /* This function is only called in debug mode. */
  bool checkJointTransforms(const JointModel* joint) const;

  /* This function is only called in debug mode. */
  bool checkLinkTransforms() const;

  /* This function is only called in debug mode. */
  bool checkCollisionTransforms() const;

  const RobotModel::ConstSharedPtr robot_model_;

  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_or_acceleration_;
  bool has_velocity_ = false;
  bool has_acceleration_ = false;
  bool has_effort_ = false;

  const JointModel* dirty_link_transforms_ = nullptr;
  const JointModel* dirty_collision_body_transforms_ = nullptr;

  std::vector<Eigen::Isometry3d> variable_joint_transforms_;  // Local transforms of all joints
  std::vector<Eigen::Isometry3d> global_link_transforms_;     // Transforms from model frame to link frame for each link
  std::vector<Eigen::Isometry3d> global_collision_body_transforms_;  // Transforms from model frame to collision bodies
  std::vector<uint8_t> dirty_joint_transforms_;
};
}  // namespace tobas
