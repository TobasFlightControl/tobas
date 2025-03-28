#pragma once

#include <cassert>
#include <rclcpp/duration.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "./robot_model.hpp"
#include "./attached_body.hpp"
#include "./transforms.hpp"

namespace tobas
{
TOBAS_CLASS_FORWARD(RobotState);  // Defines RobotStatePtr, ConstPtr, WeakPtr... etc

/**
 * @brief Signature for functions that can verify that if the group \e joint_group in \e robot_state is set to \e
 * joint_group_variable_values
 * the state is valid or not. Returns true if the state is valid. This call is allowed to modify \e robot_state (e.g.,
 * set \e joint_group_variable_values)
 */
typedef std::function<
  bool(RobotState* robot_state, const JointModelGroup* joint_group, const double* joint_group_variable_values)>
  GroupStateValidityCallbackFn;

/**
 * @brief Representation of a robot's state. This includes position, velocity, acceleration and effort.
 *
 * At the lowest level, a state is a collection of variables.
 * Each variable has a name and can have position, velocity, acceleration and effort associated to it.
 * Effort and acceleration share the memory area for efficiency reasons
 * (one should not set both acceleration and effort in the same state and expect things to work).
 * Often variables correspond to joint names as well (joints ith one degree of freedom have one variable),
 * but joints with multiple degrees of freedom have more variables.
 * Operations are allowed at variable level, joint level (see JointModel) and joint group level (see JointModelGroup).
 *
 * For efficiency reasons a state computes forward kinematics in a lazy fashion.
 * This can sometimes lead to problems if the update() function was not called on the state.
 */
class RobotState
{
public:
  /**
   * @brief A state can be constructed from a specified robot model. No values are initialized.
   * Call setToDefaultValues() if a state needs to provide valid information.
   */
  RobotState(const RobotModelConstPtr& robot_model);
  ~RobotState();

  /* Copy constructor. */
  RobotState(const RobotState& other);

  /* Copy operator */
  RobotState& operator=(const RobotState& other);

  /* Get the robot model this state is constructed for. */
  const RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  /* Get the number of variables that make up this state. */
  std::size_t getVariableCount() const
  {
    return robot_model_->getVariableCount();
  }

  /* Get the names of the variables that make up this state, in the order they are stored in memory. */
  const std::vector<std::string>& getVariableNames() const
  {
    return robot_model_->getVariableNames();
  }

  /* Get the model of a particular link */
  const LinkModel* getLinkModel(const std::string& link) const
  {
    return robot_model_->getLinkModel(link);
  }

  /* Get the model of a particular joint */
  const JointModel* getJointModel(const std::string& joint) const
  {
    return robot_model_->getJointModel(joint);
  }

  /* Get the model of a particular joint group */
  const JointModelGroup* getJointModelGroup(const std::string& group) const
  {
    return robot_model_->getJointModelGroup(group);
  }

  /** \name Getting and setting variable position
   * @{
   */

  /* Get a raw pointer to the positions of the variables
   * stored in this state. Use carefully. If you change these values
   * externally you need to make sure you trigger a forced update for
   * the state by calling update(true). */
  double* getVariablePositions()
  {
    return position_.data();
  }

  /* Get a raw pointer to the positions of the variables
      stored in this state. */
  const double* getVariablePositions() const
  {
    return position_.data();
  }

  /* It is assumed \e positions is an array containing the new
   * positions for all variables in this state. Those values are
   * copied into the state. */
  void setVariablePositions(const double* position);

  /* It is assumed \e positions is an array containing the new
   * positions for all variables in this state. Those values are
   * copied into the state. */
  void setVariablePositions(const std::vector<double>& position)
  {
    assert(robot_model_->getVariableCount() <= position.size());  // checked only in debug mode
    setVariablePositions(&position[0]);
  }

  /* Set the positions of a set of variables. If unknown variable names are specified, an exception is thrown.
   */
  void setVariablePositions(const std::map<std::string, double>& variable_map);

  /* Set the positions of a set of variables. If unknown variable names are specified, an exception is thrown.
   * Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void
  setVariablePositions(const std::map<std::string, double>& variable_map, std::vector<std::string>& missing_variables);

  /* Set the positions of a set of variables. If unknown variable names are specified, an exception is thrown.
   * Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void
  setVariablePositions(const std::vector<std::string>& variable_names, const std::vector<double>& variable_position);

  /* Set the position of a single variable. An exception is thrown if the variable name is not known */
  void setVariablePosition(const std::string& variable, double value)
  {
    setVariablePosition(robot_model_->getVariableIndex(variable), value);
  }

  /* Set the position of a single variable. The variable is specified by its index (a value associated by the
   * RobotModel to each variable) */
  void setVariablePosition(int index, double value)
  {
    position_[index] = value;
    const JointModel* jm = robot_model_->getJointOfVariable(index);
    if (jm)
    {
      markDirtyJointTransforms(jm);
      updateMimicJoint(jm);
    }
  }

  /* Get the position of a particular variable. An exception is thrown if the variable is not known. */
  double getVariablePosition(const std::string& variable) const
  {
    return position_[robot_model_->getVariableIndex(variable)];
  }

  /* Get the position of a particular variable. The variable is
   * specified by its index. No checks are performed for the validity
   * of the index passed */
  double getVariablePosition(int index) const
  {
    return position_[index];
  }

  /* By default, if velocities are never set or initialized,
   * the state remembers that there are no velocities set. This is
   * useful to know when serializing or copying the state.*/
  bool hasVelocities() const
  {
    return has_velocity_;
  }

  /* Get raw access to the velocities of the variables that make up this state. The values are in the same order
   * as reported by getVariableNames() */
  double* getVariableVelocities()
  {
    markVelocity();
    return velocity_.data();
  }

  /* Get const access to the velocities of the variables that make up this state. The values are in the same
   * order as reported by getVariableNames() */
  const double* getVariableVelocities() const
  {
    return velocity_.data();
  }

  /* Set all velocities to 0.0 */
  void zeroVelocities();

  /* Given an array with velocity values for all variables, set those values as the velocities in this state */
  void setVariableVelocities(const double* velocity)
  {
    has_velocity_ = true;
    // assume everything is in order in terms of array lengths (for efficiency reasons)
    memcpy(velocity_.data(), velocity, robot_model_->getVariableCount() * sizeof(double));
  }

  /* Given an array with velocity values for all variables, set those values as the velocities in this state */
  void setVariableVelocities(const std::vector<double>& velocity)
  {
    assert(robot_model_->getVariableCount() <= velocity.size());  // checked only in debug mode
    setVariableVelocities(&velocity[0]);
  }

  /* Set the velocities of a set of variables. If unknown variable names are specified, an exception is thrown.
   */
  void setVariableVelocities(const std::map<std::string, double>& variable_map);

  /* Set the velocities of a set of variables. If unknown variable names are specified, an exception is thrown.
   * Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void
  setVariableVelocities(const std::map<std::string, double>& variable_map, std::vector<std::string>& missing_variables);

  /* Set the velocities of a set of variables. If unknown variable names are specified, an exception is thrown.
   */
  void
  setVariableVelocities(const std::vector<std::string>& variable_names, const std::vector<double>& variable_velocity);

  /* Set the velocity of a variable. If an unknown variable name is specified, an exception is thrown. */
  void setVariableVelocity(const std::string& variable, double value)
  {
    setVariableVelocity(robot_model_->getVariableIndex(variable), value);
  }

  /* Set the velocity of a single variable. The variable is specified by its index (a value associated by the
   * RobotModel to each variable) */
  void setVariableVelocity(int index, double value)
  {
    markVelocity();
    velocity_[index] = value;
  }

  /* Get the velocity of a particular variable. An exception is thrown if the variable is not known. */
  double getVariableVelocity(const std::string& variable) const
  {
    return velocity_[robot_model_->getVariableIndex(variable)];
  }

  /* Get the velocity of a particular variable. The variable is
   * specified by its index. No checks are performed for the validity
   * of the index passed */
  double getVariableVelocity(int index) const
  {
    return velocity_[index];
  }

  /* Remove velocities from this state (this differs from setting them to zero) */
  void dropVelocities();

  /* By default, if accelerations are never set or initialized, the state remembers that there are no
   * accelerations set. This is
   * useful to know when serializing or copying the state. If hasAccelerations() reports true, hasEffort() will
   * certainly report false. */
  bool hasAccelerations() const
  {
    return has_acceleration_;
  }

  /* Get raw access to the accelerations of the variables that make up this state. The values are in the same
   * order as reported by getVariableNames(). The area of memory overlaps with effort (effort and acceleration should
   * not be set at the same time) */
  double* getVariableAccelerations()
  {
    markAcceleration();
    return effort_or_acceleration_.data();
  }

  /* Get const raw access to the accelerations of the variables that make up this state. The values are in the
   * same order as reported by getVariableNames() */
  const double* getVariableAccelerations() const
  {
    return effort_or_acceleration_.data();
  }

  /* Set all accelerations to 0.0 */
  void zeroAccelerations();

  /* Given an array with acceleration values for all variables, set those values as the accelerations in this
   * state */
  void setVariableAccelerations(const double* acceleration)
  {
    has_acceleration_ = true;
    has_effort_ = false;

    // assume everything is in order in terms of array lengths (for efficiency reasons)
    memcpy(effort_or_acceleration_.data(), acceleration, robot_model_->getVariableCount() * sizeof(double));
  }

  /* Given an array with acceleration values for all variables, set those values as the accelerations in this
   * state */
  void setVariableAccelerations(const std::vector<double>& acceleration)
  {
    assert(robot_model_->getVariableCount() <= acceleration.size());  // checked only in debug mode
    setVariableAccelerations(&acceleration[0]);
  }

  /* Set the accelerations of a set of variables. If unknown variable names are specified, an exception is
   * thrown. */
  void setVariableAccelerations(const std::map<std::string, double>& variable_map);

  /* Set the accelerations of a set of variables. If unknown variable names are specified, an exception is thrown.
   * Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void setVariableAccelerations(
    const std::map<std::string, double>& variable_map,
    std::vector<std::string>& missing_variables);

  /* Set the accelerations of a set of variables. If unknown variable names are specified, an exception is
   * thrown. */
  void setVariableAccelerations(
    const std::vector<std::string>& variable_names,
    const std::vector<double>& variable_acceleration);

  /* Set the acceleration of a variable. If an unknown variable name is specified, an exception is thrown. */
  void setVariableAcceleration(const std::string& variable, double value)
  {
    setVariableAcceleration(robot_model_->getVariableIndex(variable), value);
  }

  /* Set the acceleration of a single variable. The variable is specified by its index (a value associated by
   * the RobotModel to each variable) */
  void setVariableAcceleration(int index, double value)
  {
    markAcceleration();
    effort_or_acceleration_[index] = value;
  }

  /* Get the acceleration of a particular variable. An exception is thrown if the variable is not known. */
  double getVariableAcceleration(const std::string& variable) const
  {
    return effort_or_acceleration_[robot_model_->getVariableIndex(variable)];
  }

  /* Get the acceleration of a particular variable. The variable is
   * specified by its index. No checks are performed for the validity
   * of the index passed */
  double getVariableAcceleration(int index) const
  {
    return effort_or_acceleration_[index];
  }

  /* Remove accelerations from this state (this differs from setting them to zero) */
  void dropAccelerations();

  /* By default, if effort is never set or initialized, the state remembers that there is no effort set. This is
   * useful to know when serializing or copying the state. If hasEffort() reports true, hasAccelerations() will
   * certainly report false. */
  bool hasEffort() const
  {
    return has_effort_;
  }

  /* Get raw access to the effort of the variables that make up this state. The values are in the same order as
   * reported by getVariableNames(). The area of memory overlaps with accelerations (effort and acceleration should not
   * be set at the same time) */
  double* getVariableEffort()
  {
    markEffort();
    return effort_or_acceleration_.data();
  }

  /* Get const raw access to the effort of the variables that make up this state. The values are in the same
   * order as reported by getVariableNames(). */
  const double* getVariableEffort() const
  {
    return effort_or_acceleration_.data();
  }

  /* Set all effort values to 0.0 */
  void zeroEffort();

  /* Given an array with effort values for all variables, set those values as the effort in this state */
  void setVariableEffort(const double* effort)
  {
    has_effort_ = true;
    has_acceleration_ = false;
    // assume everything is in order in terms of array lengths (for efficiency reasons)
    memcpy(effort_or_acceleration_.data(), effort, robot_model_->getVariableCount() * sizeof(double));
  }

  /* Given an array with effort values for all variables, set those values as the effort in this state */
  void setVariableEffort(const std::vector<double>& effort)
  {
    assert(robot_model_->getVariableCount() <= effort.size());  // checked only in debug mode
    setVariableEffort(&effort[0]);
  }

  /* Set the effort of a set of variables. If unknown variable names are specified, an exception is thrown. */
  void setVariableEffort(const std::map<std::string, double>& variable_map);

  /* Set the effort of a set of variables. If unknown variable names are specified, an exception is thrown.
   * Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void
  setVariableEffort(const std::map<std::string, double>& variable_map, std::vector<std::string>& missing_variables);

  /* Set the effort of a set of variables. If unknown variable names are specified, an exception is thrown. */
  void
  setVariableEffort(const std::vector<std::string>& variable_names, const std::vector<double>& variable_acceleration);

  /* Set the effort of a variable. If an unknown variable name is specified, an exception is thrown. */
  void setVariableEffort(const std::string& variable, double value)
  {
    setVariableEffort(robot_model_->getVariableIndex(variable), value);
  }

  /* Set the effort of a single variable. The variable is specified by its index (a value associated by the
   * RobotModel to each variable) */
  void setVariableEffort(int index, double value)
  {
    markEffort();
    effort_or_acceleration_[index] = value;
  }

  /* Get the effort of a particular variable. An exception is thrown if the variable is not known. */
  double getVariableEffort(const std::string& variable) const
  {
    return effort_or_acceleration_[robot_model_->getVariableIndex(variable)];
  }

  /* Get the effort of a particular variable. The variable is
   * specified by its index. No checks are performed for the validity of the index passed. */
  double getVariableEffort(int index) const
  {
    return effort_or_acceleration_[index];
  }

  /* Remove effort values from this state (this differs from setting them to zero) */
  void dropEffort();

  /* Reduce RobotState to kinematic information (remove velocity, acceleration and effort, if present) */
  void dropDynamics();

  /* Invert velocity if present. */
  void invertVelocity();

  /** \name Getting and setting joint positions, velocities, accelerations and effort for a single joint
   * The joint might be multi-DOF, i.e. require more than one variable to set.
   * See setVariablePositions(), setVariableVelocities(), setVariableEffort() to handle multiple joints.
   */
  void setJointPositions(const std::string& joint_name, const double* position)
  {
    setJointPositions(robot_model_->getJointModel(joint_name), position);
  }

  void setJointPositions(const std::string& joint_name, const std::vector<double>& position)
  {
    setJointPositions(robot_model_->getJointModel(joint_name), &position[0]);
  }

  void setJointPositions(const JointModel* joint, const std::vector<double>& position)
  {
    setJointPositions(joint, &position[0]);
  }

  void setJointPositions(const JointModel* joint, const double* position);

  void setJointPositions(const std::string& joint_name, const Eigen::Isometry3d& transform)
  {
    setJointPositions(robot_model_->getJointModel(joint_name), transform);
  }

  void setJointPositions(const JointModel* joint, const Eigen::Isometry3d& transform);

  void setJointVelocities(const JointModel* joint, const double* velocity);

  void setJointEfforts(const JointModel* joint, const double* effort);

  const double* getJointPositions(const std::string& joint_name) const
  {
    return getJointPositions(robot_model_->getJointModel(joint_name));
  }

  /* Returns nullptr if `joint` doesn't have any active variables. */
  const double* getJointPositions(const JointModel* joint) const;

  const double* getJointVelocities(const std::string& joint_name) const
  {
    return getJointVelocities(robot_model_->getJointModel(joint_name));
  }

  /* Returns nullptr if `joint` doesn't have any active variables. */
  const double* getJointVelocities(const JointModel* joint) const;

  const double* getJointAccelerations(const std::string& joint_name) const
  {
    return getJointAccelerations(robot_model_->getJointModel(joint_name));
  }

  /* Returns nullptr if `joint` doesn't have any active variables. */
  const double* getJointAccelerations(const JointModel* joint) const;

  const double* getJointEffort(const std::string& joint_name) const
  {
    return getJointEffort(robot_model_->getJointModel(joint_name));
  }

  /* Returns nullptr if `joint` doesn't have any active variables. */
  const double* getJointEffort(const JointModel* joint) const;

  /* Given positions for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupPositions(const std::string& joint_group_name, const double* gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupPositions(jmg, gstate);
  }

  /* Given positions for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupPositions(const std::string& joint_group_name, const std::vector<double>& gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
    {
      assert(gstate.size() == jmg->getVariableCount());
      setJointGroupPositions(jmg, &gstate[0]);
    }
  }

  /* Given positions for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupPositions(const JointModelGroup* group, const std::vector<double>& gstate)
  {
    assert(gstate.size() == group->getVariableCount());
    setJointGroupPositions(group, &gstate[0]);
  }

  /* Given positions for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupPositions(const JointModelGroup* group, const double* gstate);

  /* Given positions for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupPositions(const std::string& joint_group_name, const Eigen::VectorXd& values)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
    {
      assert(values.size() == jmg->getVariableCount());
      setJointGroupPositions(jmg, values);
    }
  }

  /* Given positions for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupPositions(const JointModelGroup* group, const Eigen::VectorXd& values);

  /* Given positions for the variables of active joints that make up a group,
   * in the order found in the group (excluding values of mimic joints), set those
   * as the new values that correspond to the group */
  void setJointGroupActivePositions(const JointModelGroup* group, const std::vector<double>& gstate);

  /* Given positions for the variables of active joints that make up a group,
   * in the order found in the group (excluding values of mimic joints), set those
   * as the new values that correspond to the group */
  void setJointGroupActivePositions(const std::string& joint_group_name, const std::vector<double>& gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
    {
      assert(gstate.size() == jmg->getActiveVariableCount());
      setJointGroupActivePositions(jmg, gstate);
    }
  }

  /* Given positions for the variables of active joints that make up a group,
   * in the order found in the group (excluding values of mimic joints), set those
   * as the new values that correspond to the group */
  void setJointGroupActivePositions(const JointModelGroup* group, const Eigen::VectorXd& values);

  /* Given positions for the variables of active joints that make up a group,
   * in the order found in the group (excluding values of mimic joints), set those
   * as the new values that correspond to the group */
  void setJointGroupActivePositions(const std::string& joint_group_name, const Eigen::VectorXd& values)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
    {
      assert(values.size() == jmg->getActiveVariableCount());
      setJointGroupActivePositions(jmg, values);
    }
  }

  /* For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const std::string& joint_group_name, std::vector<double>& gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
    {
      gstate.resize(jmg->getVariableCount());
      copyJointGroupPositions(jmg, &gstate[0]);
    }
  }

  /* For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const std::string& joint_group_name, double* gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupPositions(jmg, gstate);
  }

  /* For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const JointModelGroup* group, std::vector<double>& gstate) const
  {
    gstate.resize(group->getVariableCount());
    copyJointGroupPositions(group, &gstate[0]);
  }

  /* For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const JointModelGroup* group, double* gstate) const;

  /* For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const std::string& joint_group_name, Eigen::VectorXd& values) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupPositions(jmg, values);
  }

  /* For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const JointModelGroup* group, Eigen::VectorXd& values) const;

  /* Given velocities for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupVelocities(const std::string& joint_group_name, const double* gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupVelocities(jmg, gstate);
  }

  /* Given velocities for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupVelocities(const std::string& joint_group_name, const std::vector<double>& gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupVelocities(jmg, &gstate[0]);
  }

  /* Given velocities for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupVelocities(const JointModelGroup* group, const std::vector<double>& gstate)
  {
    setJointGroupVelocities(group, &gstate[0]);
  }

  /* Given velocities for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupVelocities(const JointModelGroup* group, const double* gstate);

  /* Given velocities for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupVelocities(const std::string& joint_group_name, const Eigen::VectorXd& values)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupVelocities(jmg, values);
  }

  /* Given velocities for the variables that make up a group, in the order found in the group (including values
   * of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupVelocities(const JointModelGroup* group, const Eigen::VectorXd& values);

  /* For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const std::string& joint_group_name, std::vector<double>& gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
    {
      gstate.resize(jmg->getVariableCount());
      copyJointGroupVelocities(jmg, &gstate[0]);
    }
  }

  /* For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const std::string& joint_group_name, double* gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupVelocities(jmg, gstate);
  }

  /* For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const JointModelGroup* group, std::vector<double>& gstate) const
  {
    gstate.resize(group->getVariableCount());
    copyJointGroupVelocities(group, &gstate[0]);
  }

  /* For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const JointModelGroup* group, double* gstate) const;

  /* For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const std::string& joint_group_name, Eigen::VectorXd& values) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupVelocities(jmg, values);
  }

  /* For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const JointModelGroup* group, Eigen::VectorXd& values) const;

  /* Given accelerations for the variables that make up a group, in the order found in the group (including
   * values of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupAccelerations(const std::string& joint_group_name, const double* gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupAccelerations(jmg, gstate);
  }

  /* Given accelerations for the variables that make up a group, in the order found in the group (including
   * values of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupAccelerations(const std::string& joint_group_name, const std::vector<double>& gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupAccelerations(jmg, &gstate[0]);
  }

  /* Given accelerations for the variables that make up a group, in the order found in the group (including
   * values of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupAccelerations(const JointModelGroup* group, const std::vector<double>& gstate)
  {
    setJointGroupAccelerations(group, &gstate[0]);
  }

  /* Given accelerations for the variables that make up a group, in the order found in the group (including
   * values of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupAccelerations(const JointModelGroup* group, const double* gstate);

  /* Given accelerations for the variables that make up a group, in the order found in the group (including
   * values of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupAccelerations(const std::string& joint_group_name, const Eigen::VectorXd& values)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupAccelerations(jmg, values);
  }

  /* Given accelerations for the variables that make up a group, in the order found in the group (including
   * values of mimic joints), set those as the new values that correspond to the group */
  void setJointGroupAccelerations(const JointModelGroup* group, const Eigen::VectorXd& values);

  /* For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const std::string& joint_group_name, std::vector<double>& gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
    {
      gstate.resize(jmg->getVariableCount());
      copyJointGroupAccelerations(jmg, &gstate[0]);
    }
  }

  /* For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const std::string& joint_group_name, double* gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupAccelerations(jmg, gstate);
  }

  /* For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const JointModelGroup* group, std::vector<double>& gstate) const
  {
    gstate.resize(group->getVariableCount());
    copyJointGroupAccelerations(group, &gstate[0]);
  }

  /* For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const JointModelGroup* group, double* gstate) const;

  /* For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const std::string& joint_group_name, Eigen::VectorXd& values) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupAccelerations(jmg, values);
  }

  /* For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const JointModelGroup* group, Eigen::VectorXd& values) const;

  /**
   * @brief If the group this state corresponds to is a chain and a solver is available,
   * then the joint values can be set by computing inverse kinematics.
   * The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
   *
   * @param pose The pose the last link in the chain needs to achieve
   * @param timeout The timeout passed to the kinematics solver on each attempt
   * @param constraint A state validity constraint to be required for IK solutions
   */
  bool setFromIK(
    const JointModelGroup* group,
    const geometry_msgs::msg::Pose& pose,
    double timeout = 0.0,
    const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
    const KinematicsQueryOptions& options = KinematicsQueryOptions(),
    const KinematicsBase::IKCostFn& cost_function = KinematicsBase::IKCostFn());

  /**
   * @brief If the group this state corresponds to is a chain and a solver is available,
   * then the joint values can be set by computing inverse kinematics.
   * The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
   *
   * @param pose The pose the \e tip  link in the chain needs to achieve
   * @param tip The name of the link the pose is specified for
   * @param timeout The timeout passed to the kinematics solver on each attempt
   * @param constraint A state validity constraint to be required for IK solutions
   */
  bool setFromIK(
    const JointModelGroup* group,
    const geometry_msgs::msg::Pose& pose,
    const std::string& tip,
    double timeout = 0.0,
    const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
    const KinematicsQueryOptions& options = KinematicsQueryOptions(),
    const KinematicsBase::IKCostFn& cost_function = KinematicsBase::IKCostFn());

  /**
   * @brief If the group this state corresponds to is a chain and a solver is available,
   * then the joint values can be eset by computing inverse kinematics.
   * The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
   *
   * @param pose The pose the last link in the chain needs to achieve
   * @param tip The name of the link the pose is specified for
   * @param timeout The timeout passed to the kinematics solver on each attempt
   */
  bool setFromIK(
    const JointModelGroup* group,
    const Eigen::Isometry3d& pose,
    double timeout = 0.0,
    const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
    const KinematicsQueryOptions& options = KinematicsQueryOptions(),
    const KinematicsBase::IKCostFn& cost_function = KinematicsBase::IKCostFn());

  /**
   * @brief If the group this state corresponds to is a chain and a solver is available,
   * then the joint values can be set by computing inverse kinematics.
   * The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
   *
   * @param pose The pose the last link in the chain needs to achieve
   * @param timeout The timeout passed to the kinematics solver on each attempt
   * @param constraint A state validity constraint to be required for IK solutions
   */
  bool setFromIK(
    const JointModelGroup* group,
    const Eigen::Isometry3d& pose,
    const std::string& tip,
    double timeout = 0.0,
    const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
    const KinematicsQueryOptions& options = KinematicsQueryOptions(),
    const KinematicsBase::IKCostFn& cost_function = KinematicsBase::IKCostFn());

  /**
   * @brief If the group this state corresponds to is a chain and a solver is available,
   * then the joint values can be set by computing inverse kinematics.
   * The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
   *
   * @param pose The pose the last link in the chain needs to achieve
   * @param tip The name of the frame for which IK is attempted.
   * @param consistency_limits This specifies the desired distance between the solution and the seed state
   * @param timeout The timeout passed to the kinematics solver on each attempt
   * @param constraint A state validity constraint to be required for IK solutions
   */
  bool setFromIK(
    const JointModelGroup* group,
    const Eigen::Isometry3d& pose,
    const std::string& tip,
    const std::vector<double>& consistency_limits,
    double timeout = 0.0,
    const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
    const KinematicsQueryOptions& options = KinematicsQueryOptions(),
    const KinematicsBase::IKCostFn& cost_function = KinematicsBase::IKCostFn());

  /**
   * @brief Warning: This function inefficiently copies all transforms around.
   * If the group consists of a set of sub-groups that are each a chain and a solver
   * is available for each sub-group, then the joint values can be set by computing inverse kinematics.
   * The poses are assumed to be in the reference frame of the kinematic model. The poses are assumed
   * to be in the same order as the order of the sub-groups in this group. Returns true on success.
   *
   * @param poses The poses the last link in each chain needs to achieve
   * @param tips The names of the frames for which IK is attempted.
   * @param timeout The timeout passed to the kinematics solver on each attempt
   * @param constraint A state validity constraint to be required for IK solutions
   */
  bool setFromIK(
    const JointModelGroup* group,
    const EigenSTL::vector_Isometry3d& poses,
    const std::vector<std::string>& tips,
    double timeout = 0.0,
    const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
    const KinematicsQueryOptions& options = KinematicsQueryOptions(),
    const KinematicsBase::IKCostFn& cost_function = KinematicsBase::IKCostFn());

  /**
   * @brief Warning: This function inefficiently copies all transforms around.
   * If the group consists of a set of sub-groups that are each a chain and a solver
   * is available for each sub-group, then the joint values can be set by computing inverse kinematics.
   * The poses are assumed to be in the reference frame of the kinematic model. The poses are assumed
   * to be in the same order as the order of the sub-groups in this group. Returns true on success.
   *
   * @param poses The poses the last link in each chain needs to achieve
   * @param tips The names of the frames for which IK is attempted.
   * @param consistency_limits This specifies the desired distance between the solution and the seed state
   * @param timeout The timeout passed to the kinematics solver on each attempt
   * @param constraint A state validity constraint to be required for IK solutions
   */
  bool setFromIK(
    const JointModelGroup* group,
    const EigenSTL::vector_Isometry3d& poses,
    const std::vector<std::string>& tips,
    const std::vector<std::vector<double>>& consistency_limits,
    double timeout = 0.0,
    const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
    const KinematicsQueryOptions& options = KinematicsQueryOptions(),
    const KinematicsBase::IKCostFn& cost_function = KinematicsBase::IKCostFn());

  /**
   * @brief setFromIK for multiple poses and tips (end effectors) when no solver exists for the jmg that can solver for
   * non-chain kinematics. In this case, we divide the group into subgroups and do IK solving individually
   *
   * @param poses The poses the last link in each chain needs to achieve
   * @param tips The names of the frames for which IK is attempted.
   * @param consistency_limits This specifies the desired distance between the solution and the seed state
   * @param timeout The timeout passed to the kinematics solver on each attempt
   * @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIKSubgroups(
    const JointModelGroup* group,
    const EigenSTL::vector_Isometry3d& poses,
    const std::vector<std::string>& tips,
    const std::vector<std::vector<double>>& consistency_limits,
    double timeout = 0.0,
    const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
    const KinematicsQueryOptions& options = KinematicsQueryOptions());

  /**
   * @brief Set the joint values from a Cartesian velocity applied during a time dt
   *
   * @param group the group of joints this function operates on
   * @param twist a Cartesian velocity on the 'tip' frame
   * @param tip the frame for which the twist is given
   * @param dt a time interval (seconds)
   * @param st a secondary task computation function
   */
  bool setFromDiffIK(
    const JointModelGroup* group,
    const Eigen::VectorXd& twist,
    const std::string& tip,
    double dt,
    const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn());

  /**
   * @brief Set the joint values from a Cartesian velocity applied during a time dt
   *
   * @param group the group of joints this function operates on
   * @param twist a Cartesian velocity on the 'tip' frame
   * @param tip the frame for which the twist is given
   * @param dt a time interval (seconds)
   * @param st a secondary task computation function
   */
  bool setFromDiffIK(
    const JointModelGroup* group,
    const geometry_msgs::msg::Twist& twist,
    const std::string& tip,
    double dt,
    const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn());

  /**
   * @brief Compute the Jacobian with reference to a particular point on a given link, for a specified group.
   *
   * @param group The group to compute the Jacobian for
   * @param link The link model to compute the Jacobian for
   * @param reference_point_position The reference point position (with respect to the link specified in link)
   * @param jacobian The resultant jacobian, with the origin at the group root link.
   * @param use_quaternion_representation Flag indicating if the Jacobian should use a quaternion representation
   * (default is false)
   *
   * @return True if jacobian was successfully computed, false otherwise
   */
  bool getJacobian(
    const JointModelGroup* group,
    const LinkModel* link,
    const Eigen::Vector3d& reference_point_position,
    Eigen::MatrixXd& jacobian,
    bool use_quaternion_representation = false) const;

  /**
   * @brief Compute the Jacobian with reference to a particular point on a given link, for a specified group.
   *
   * @param group The group to compute the Jacobian for
   * @param link The link model to compute the Jacobian for
   * @param reference_point_position The reference point position (with respect to the link specified in link)
   * @param jacobian The resultant jacobian, with the origin at the group root link.
   * @param use_quaternion_representation Flag indicating if the Jacobian should use a quaternion representation
   * (default is false)
   *
   * @return True if jacobian was successfully computed, false otherwise
   */
  bool getJacobian(
    const JointModelGroup* group,
    const LinkModel* link,
    const Eigen::Vector3d& reference_point_position,
    Eigen::MatrixXd& jacobian,
    bool use_quaternion_representation = false)
  {
    updateLinkTransforms();
    return static_cast<const RobotState*>(this)->getJacobian(
      group, link, reference_point_position, jacobian, use_quaternion_representation);
  }

  /**
   * @brief Compute the Jacobian with reference to the last link of a specified group,
   * and origin at the group root link.
   * If the group is not a chain, an exception is thrown.
   *
   * @param group The group to compute the Jacobian for
   * @param reference_point_position The reference point position (with respect to the link specified in link_name)
   *
   * @return The computed Jacobian.
   */
  Eigen::MatrixXd getJacobian(
    const JointModelGroup* group,
    const Eigen::Vector3d& reference_point_position = Eigen::Vector3d(0.0, 0.0, 0.0)) const;

  /**
   * @brief Compute the Jacobian with reference to the last link of a specified group,
   * and origin at the group root link.
   * If the group is not a chain, an exception is thrown.
   *
   * @param group The group to compute the Jacobian for
   * @param reference_point_position The reference point position (with respect to the link specified in link_name)
   *
   * @return The computed Jacobian.
   */
  Eigen::MatrixXd getJacobian(
    const JointModelGroup* group,
    const Eigen::Vector3d& reference_point_position = Eigen::Vector3d(0.0, 0.0, 0.0))
  {
    updateLinkTransforms();
    return static_cast<const RobotState*>(this)->getJacobian(group, reference_point_position);
  }

  /* Given a twist for a particular link (\e tip), compute the corresponding velocity for every variable and
   * store it in \e qdot */
  void computeVariableVelocity(
    const JointModelGroup* jmg,
    Eigen::VectorXd& qdot,
    const Eigen::VectorXd& twist,
    const LinkModel* tip) const;

  /* Given a twist for a particular link (\e tip), compute the corresponding velocity for every variable and
   * store it in \e qdot */
  void computeVariableVelocity(
    const JointModelGroup* jmg,
    Eigen::VectorXd& qdot,
    const Eigen::VectorXd& twist,
    const LinkModel* tip)
  {
    updateLinkTransforms();
    static_cast<const RobotState*>(this)->computeVariableVelocity(jmg, qdot, twist, tip);
  }

  /* Given the velocities for the variables in this group (\e qdot) and an amount of time (\e dt),
      update the current state using the Euler forward method. If the constraint specified is satisfied, return true,
     otherwise return false. */
  bool integrateVariableVelocity(
    const JointModelGroup* jmg,
    const Eigen::VectorXd& qdot,
    double dt,
    const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn());

  void setVariableValues(const sensor_msgs::msg::JointState& msg)
  {
    if (!msg.position.empty())
      setVariablePositions(msg.name, msg.position);
    if (!msg.velocity.empty())
      setVariableVelocities(msg.name, msg.velocity);
  }

  /* Set all joints to their default positions.
   * The default position is 0, or if that is not within bounds then half way
   * between min and max bound. */
  void setToDefaultValues();

  /* Set the joints in \e group to the position \e name defined in the SRDF */
  bool setToDefaultValues(const JointModelGroup* group, const std::string& name);

  bool setToDefaultValues(const std::string& group_name, const std::string& state_name)
  {
    const JointModelGroup* jmg = getJointModelGroup(group_name);
    if (jmg)
    {
      return setToDefaultValues(jmg, state_name);
    }
    else
    {
      return false;
    }
  }

  /* Set all joints to random values.  Values will be within default bounds. */
  void setToRandomPositions();

  /* Set all joints in \e group to random values.  Values will be within default bounds. */
  void setToRandomPositions(const JointModelGroup* group);

  /* Set all joints in \e group to random values using a specified random number generator.
      Values will be within default bounds. */
  void setToRandomPositions(const JointModelGroup* group, random_numbers::RandomNumberGenerator& rng);

  /* Set all joints in \e group to random values near the value in \e seed.
   * \e distance is the maximum amount each joint value will vary from the
   * corresponding value in \e seed.  \distance represents meters for
   * prismatic/positional joints and radians for revolute/orientation joints.
   * Resulting values are clamped within default bounds. */
  void setToRandomPositionsNearBy(const JointModelGroup* group, const RobotState& seed, double distance);

  /* Set all joints in \e group to random values near the value in \e seed, using a specified random number
   * generator. \e distance is the maximum amount each joint value will vary from the corresponding value in \e seed.
   * \distance represents meters for prismatic/positional joints and radians for revolute/orientation joints. Resulting
   * values are clamped within default bounds. */
  void setToRandomPositionsNearBy(
    const JointModelGroup* group,
    const RobotState& seed,
    double distance,
    random_numbers::RandomNumberGenerator& rng);

  /* Set all joints in \e group to random values near the value in \e seed.
   * \e distances \b MUST have the same size as \c
   * group.getActiveJointModels().  Each value in \e distances is the maximum
   * amount the corresponding active joint in \e group will vary from the
   * corresponding value in \e seed.  \distance represents meters for
   * prismatic/positional joints and radians for revolute/orientation joints.
   * Resulting values are clamped within default bounds. */
  void setToRandomPositionsNearBy(
    const JointModelGroup* group,
    const RobotState& seed,
    const std::vector<double>& distances);

  /* Set all joints in \e group to random values near the value in \e seed, using a specified random number
   * generator. \e distances \b MUST have the same size as \c group.getActiveJointModels().  Each value in \e distances
   * is the maximum amount the corresponding active joint in \e group will vary from the corresponding value in \e seed.
   * \distance represents meters for prismatic/positional joints and radians for revolute/orientation joints. Resulting
   * values are clamped within default bounds. */
  void setToRandomPositionsNearBy(
    const JointModelGroup* group,
    const RobotState& seed,
    const std::vector<double>& distances,
    random_numbers::RandomNumberGenerator& rng);

  /* Update the transforms for the collision bodies. This call is needed before calling collision checking.
   * If updating link transforms or joint transforms is needed, the corresponding updates are also triggered. */
  void updateCollisionBodyTransforms();

  /* Update the reference frame transforms for links. This call is needed before using the transforms of links
   * for coordinate transforms. */
  void updateLinkTransforms();

  /* Update all transforms. */
  void update(bool force = false);

  /* Update the state after setting a particular link to the input global transform pose.
   *
   * This "warps" the given link to the given pose, neglecting the joint values of its parent joint.
   * The link transforms of link and all its descendants are updated, but not marked as dirty,
   * although they do not match the joint values anymore!
   * Collision body transforms are not yet updated, but marked dirty only.
   * Use update(false) or updateCollisionBodyTransforms() to update them as well.
   */
  void updateStateWithLinkAt(const std::string& link_name, const Eigen::Isometry3d& transform, bool backward = false)
  {
    updateStateWithLinkAt(robot_model_->getLinkModel(link_name), transform, backward);
  }

  /* Update the state after setting a particular link to the input global transform pose.*/
  void updateStateWithLinkAt(const LinkModel* link, const Eigen::Isometry3d& transform, bool backward = false);

  /* Get the latest link upwards the kinematic tree which is only connected via fixed joints.
   *
   * This behaves the same as RobotModel::getRigidlyConnectedParentLinkModel,
   * but can additionally resolve parents for attached objects / subframes.
   */
  const LinkModel*
  getRigidlyConnectedParentLinkModel(const std::string& frame, const JointModelGroup* jmg = nullptr) const;

  /* Get the link transform w.r.t. the root link (model frame) of the RobotModel.
   * This is typically the root link of the URDF unless a virtual joint is present.
   * Checks the cache and if there are any dirty (non-updated) transforms, first updates them as needed.
   * A related, more comprehensive function is |getFrameTransform|, which additionally to link frames
   * also searches for attached object frames and their subframes.
   *
   * This will throw an exception if the passed link is not found
   *
   * The returned transformation is always a valid isometry.
   */
  const Eigen::Isometry3d& getGlobalLinkTransform(const std::string& link_name)
  {
    return getGlobalLinkTransform(robot_model_->getLinkModel(link_name));
  }

  const Eigen::Isometry3d& getGlobalLinkTransform(const LinkModel* link)
  {
    if (!link)
    {
      throw Exception("Invalid link");
    }
    updateLinkTransforms();
    return global_link_transforms_[link->getLinkIndex()];
  }

  const Eigen::Isometry3d& getGlobalLinkTransform(const std::string& link_name) const
  {
    return getGlobalLinkTransform(robot_model_->getLinkModel(link_name));
  }

  const Eigen::Isometry3d& getGlobalLinkTransform(const LinkModel* link) const
  {
    if (!link)
    {
      throw Exception("Invalid link");
    }
    assert(checkLinkTransforms());
    return global_link_transforms_[link->getLinkIndex()];
  }

  /* Get the link transform w.r.t. the root link (model frame) of the RobotModel.
   * This is typically the root link of the URDF unless a virtual joint is present.
   * Checks the cache and if there are any dirty (non-updated) transforms, first updates them as needed.
   *
   * As opposed to the visual links in |getGlobalLinkTransform|, this function returns
   * the collision link transform used for collision checking.
   *
   * @param link_name: name of link to lookup
   * @param index: specify which collision body to lookup, if more than one exists
   */
  const Eigen::Isometry3d& getCollisionBodyTransform(const std::string& link_name, std::size_t index)
  {
    return getCollisionBodyTransform(robot_model_->getLinkModel(link_name), index);
  }

  const Eigen::Isometry3d& getCollisionBodyTransform(const LinkModel* link, std::size_t index)
  {
    updateCollisionBodyTransforms();
    return global_collision_body_transforms_[link->getFirstCollisionBodyTransformIndex() + index];
  }

  const Eigen::Isometry3d& getCollisionBodyTransform(const std::string& link_name, std::size_t index) const
  {
    return getCollisionBodyTransform(robot_model_->getLinkModel(link_name), index);
  }

  const Eigen::Isometry3d& getCollisionBodyTransform(const LinkModel* link, std::size_t index) const
  {
    assert(checkCollisionTransforms());
    return global_collision_body_transforms_[link->getFirstCollisionBodyTransformIndex() + index];
  }

  const Eigen::Isometry3d& getJointTransform(const std::string& joint_name)
  {
    return getJointTransform(robot_model_->getJointModel(joint_name));
  }

  const Eigen::Isometry3d& getJointTransform(const JointModel* joint);

  const Eigen::Isometry3d& getJointTransform(const std::string& joint_name) const
  {
    return getJointTransform(robot_model_->getJointModel(joint_name));
  }

  const Eigen::Isometry3d& getJointTransform(const JointModel* joint) const
  {
    assert(checkJointTransforms(joint));
    return variable_joint_transforms_[joint->getJointIndex()];
  }

  bool dirtyJointTransform(const JointModel* joint) const
  {
    return dirty_joint_transforms_[joint->getJointIndex()];
  }

  bool dirtyLinkTransforms() const
  {
    return dirty_link_transforms_;
  }

  bool dirtyCollisionBodyTransforms() const
  {
    return dirty_link_transforms_ || dirty_collision_body_transforms_;
  }

  /* Returns true if anything in this state is dirty */
  bool dirty() const
  {
    return dirtyCollisionBodyTransforms();
  }

  /* Return the sum of joint distances to "other" state. An L1 norm. Only considers active joints. */
  double distance(const RobotState& other) const
  {
    return robot_model_->distance(position_.data(), other.getVariablePositions());
  }

  /* Return the sum of joint distances to "other" state. An L1 norm. Only considers active joints. */
  double distance(const RobotState& other, const JointModelGroup* joint_group) const;

  /* Return the sum of joint distances to "other" state. An L1 norm. Only considers active joints. */
  double distance(const RobotState& other, const JointModel* joint) const;

  /**
   * Interpolate towards "to" state. Mimic joints are correctly updated and flags are set so that FK is recomputed
   * when needed.
   *
   * @param to interpolate to this state
   * @param t a fraction in the range [0 1]. If 1, the result matches "to" state exactly.
   * @param state holds the result
   */
  void interpolate(const RobotState& to, double t, RobotState& state) const;

  /**
   * Interpolate towards "to" state, but only for the joints in the specified group. Mimic joints are correctly updated
   * and flags are set so that FK is recomputed when needed.
   *
   * @param to interpolate to this state
   * @param t a fraction in the range [0 1]. If 1, the result matches "to" state exactly.
   * @param state holds the result
   * @param joint_group interpolate only for the joints in this group
   */
  void interpolate(const RobotState& to, double t, RobotState& state, const JointModelGroup* joint_group) const;

  /**
   * Interpolate towards "to" state, but only for a single joint. Mimic joints are correctly updated
   * and flags are set so that FK is recomputed when needed.
   *
   * @param to interpolate to this state
   * @param t a fraction in the range [0 1]. If 1, the result matches "to" state exactly.
   * @param state holds the result
   * @param joint interpolate only for this joint
   */
  void interpolate(const RobotState& to, double t, RobotState& state, const JointModel* joint) const;

  void enforceBounds();
  void enforceBounds(const JointModelGroup* joint_group);
  void enforceBounds(const JointModel* joint)
  {
    enforcePositionBounds(joint);
    if (has_velocity_)
      enforceVelocityBounds(joint);
  }
  void enforcePositionBounds(const JointModel* joint);

  // Call harmonizePosition() for all joints / all joints in group / given joint
  void harmonizePositions();
  void harmonizePositions(const JointModelGroup* joint_group);
  void harmonizePosition(const JointModel* joint);

  void enforceVelocityBounds(const JointModel* joint);

  bool satisfiesBounds(double margin = 0.0) const;
  bool satisfiesBounds(const JointModelGroup* joint_group, double margin = 0.0) const;
  bool satisfiesBounds(const JointModel* joint, double margin = 0.0) const
  {
    return satisfiesPositionBounds(joint, margin) && (!has_velocity_ || satisfiesVelocityBounds(joint, margin));
  }
  bool satisfiesPositionBounds(const JointModel* joint, double margin = 0.0) const
  {
    return joint->satisfiesPositionBounds(getJointPositions(joint), margin);
  }
  bool satisfiesVelocityBounds(const JointModel* joint, double margin = 0.0) const
  {
    return joint->satisfiesVelocityBounds(getJointVelocities(joint), margin);
  }

  /* Get the minimm distance from this state to the bounds.
      The minimum distance and the joint for which this minimum is achieved are returned. */
  std::pair<double, const JointModel*> getMinDistanceToPositionBounds() const;

  /* Get the minimm distance from a group in this state to the bounds.
      The minimum distance and the joint for which this minimum is achieved are returned. */
  std::pair<double, const JointModel*> getMinDistanceToPositionBounds(const JointModelGroup* group) const;

  /* Get the minimm distance from a set of joints in the state to the bounds.
      The minimum distance and the joint for which this minimum is achieved are returned. */
  std::pair<double, const JointModel*>
  getMinDistanceToPositionBounds(const std::vector<const JointModel*>& joints) const;

  /**
   * @brief Check that the time to move between two waypoints is sufficient given velocity limits and time step
   * @param other - robot state to compare joint positions against
   * @param group - planning group to compare joint positions against
   * @param dt - time step between the two points
   */
  bool isValidVelocityMove(const RobotState& other, const JointModelGroup* group, double dt) const;

  /* Add an attached body to this state.
   *
   * This only adds the given body to this RobotState
   * instance.  It does not change anything about other
   * representations of the object elsewhere in the system.  So if the
   * body represents an object in a collision_detection::World (like
   * from a planning_scene::PlanningScene), you will likely need to remove the
   * corresponding object from that world to avoid having collisions
   * detected against it.
   **/
  void attachBody(std::unique_ptr<AttachedBody> attached_body);

  /* Add an attached body to a link
   * @param id The string id associated with the attached body
   * @param pose The pose associated with the attached body
   * @param shapes The shapes that make up the attached body
   * @param shape_poses The transforms between the object pose and the attached body's shapes
   * @param touch_links The set of links that the attached body is allowed to touch
   * @param link_name The link to attach to
   * @param detach_posture The posture of the gripper when placing the object
   * @param subframe_poses Transforms to points of interest on the object (can be used as end effector link)
   *
   * This only adds the given body to this RobotState
   * instance.  It does not change anything about other
   * representations of the object elsewhere in the system.  So if the
   * body represents an object in a collision_detection::World (like
   * from a planning_scene::PlanningScene), you will likely need to remove the
   * corresponding object from that world to avoid having collisions
   * detected against it. */
  void attachBody(
    const std::string& id,
    const Eigen::Isometry3d& pose,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const EigenSTL::vector_Isometry3d& shape_poses,
    const std::set<std::string>& touch_links,
    const std::string& link_name,
    const trajectory_msgs::msg::JointTrajectory& detach_posture = trajectory_msgs::msg::JointTrajectory(),
    const FixedTransformsMap& subframe_poses = FixedTransformsMap());

  /* Add an attached body to a link
   * @param id The string id associated with the attached body
   * @param pose The pose associated with the attached body
   * @param shapes The shapes that make up the attached body
   * @param shape_poses The transforms between the object pose and the attached body's shapes
   * @param touch_links The set of links that the attached body is allowed to touch
   * @param link_name The link to attach to
   * @param detach_posture The posture of the gripper when placing the object
   * @param subframe_poses Transforms to points of interest on the object (can be used as end effector link)
   *
   * This only adds the given body to this RobotState
   * instance.  It does not change anything about other
   * representations of the object elsewhere in the system.  So if the
   * body represents an object in a collision_detection::World (like
   * from a planning_scene::PlanningScene), you will likely need to remove the
   * corresponding object from that world to avoid having collisions
   * detected against it. */
  void attachBody(
    const std::string& id,
    const Eigen::Isometry3d& pose,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const EigenSTL::vector_Isometry3d& shape_poses,
    const std::vector<std::string>& touch_links,
    const std::string& link_name,
    const trajectory_msgs::msg::JointTrajectory& detach_posture = trajectory_msgs::msg::JointTrajectory(),
    const FixedTransformsMap& subframe_poses = FixedTransformsMap())
  {
    std::set<std::string> touch_links_set(touch_links.begin(), touch_links.end());
    attachBody(id, pose, shapes, shape_poses, touch_links_set, link_name, detach_posture, subframe_poses);
  }

  /* Get all bodies attached to the model corresponding to this state */
  void getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies) const;

  /* Get all bodies attached to a particular group the model corresponding to this state */
  void getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies, const JointModelGroup* group) const;

  /* Get all bodies attached to a particular link in the model corresponding to this state */
  void getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies, const LinkModel* link_model) const;

  /* Remove the attached body named \e id. Return false if the object was not found (and thus not removed).
   * Return true on success. */
  bool clearAttachedBody(const std::string& id);

  /* Clear the bodies attached to a specific link */
  void clearAttachedBodies(const LinkModel* link);

  /* Clear the bodies attached to a specific group */
  void clearAttachedBodies(const JointModelGroup* group);

  /* Clear all attached bodies. This calls delete on the AttachedBody instances, if needed. */
  void clearAttachedBodies();

  /* Get the attached body named \e name. Return nullptr if not found. */
  const AttachedBody* getAttachedBody(const std::string& name) const;

  /* Check if an attached body named \e id exists in this state */
  bool hasAttachedBody(const std::string& id) const;

  void setAttachedBodyUpdateCallback(const AttachedBodyCallback& callback);

  /* Compute an axis-aligned bounding box that contains the current state.
      The format for \e aabb is (minx, maxx, miny, maxy, minz, maxz) */
  void computeAABB(std::vector<double>& aabb) const;

  /* Compute an axis-aligned bounding box that contains the current state.
      The format for \e aabb is (minx, maxx, miny, maxy, minz, maxz) */
  void computeAABB(std::vector<double>& aabb)
  {
    updateLinkTransforms();
    static_cast<const RobotState*>(this)->computeAABB(aabb);
  }

  /* Return the instance of a random number generator */
  random_numbers::RandomNumberGenerator& getRandomNumberGenerator()
  {
    if (!rng_)
      rng_ = std::make_unique<random_numbers::RandomNumberGenerator>();
    return *rng_;
  }

  /* Get the transformation matrix from the model frame (root of model) to the frame identified by \e frame_id
   *
   * If frame_id was not found, \e frame_found is set to false and an identity transform is returned.
   *
   * The returned transformation is always a valid isometry. */
  const Eigen::Isometry3d& getFrameTransform(const std::string& frame_id, bool* frame_found = nullptr);

  /* Get the transformation matrix from the model frame (root of model) to the frame identified by \e frame_id
   *
   * If frame_id was not found, \e frame_found is set to false and an identity transform is returned.
   *
   * The returned transformation is always a valid isometry. */
  const Eigen::Isometry3d& getFrameTransform(const std::string& frame_id, bool* frame_found = nullptr) const;

  /* Get the transformation matrix from the model frame (root of model) to the frame identified by \e frame_id
   *
   * If this frame is attached to a robot link, the link pointer is returned in \e robot_link.
   * If frame_id was not found, \e frame_found is set to false and an identity transform is returned.
   *
   * The returned transformation is always a valid isometry. */
  const Eigen::Isometry3d&
  getFrameInfo(const std::string& frame_id, const LinkModel*& robot_link, bool& frame_found) const;

  /* Check if a transformation matrix from the model frame (root of model) to frame \e frame_id is known */
  bool knowsFrameTransform(const std::string& frame_id) const;

  /* Get a MarkerArray that fully describes the robot markers for a given robot.
   * @param arr The returned marker array
   * @param link_names The list of link names for which the markers should be created.
   * @param color The color for the marker
   * @param ns The namespace for the markers
   * @param dur The rclcpp::Duration for which the markers should stay visible
   */
  void getRobotMarkers(
    visualization_msgs::msg::MarkerArray& arr,
    const std::vector<std::string>& link_names,
    const std_msgs::msg::ColorRGBA& color,
    const std::string& ns,
    const rclcpp::Duration& dur,
    bool include_attached = false) const;

  /* Get a MarkerArray that fully describes the robot markers for a given robot. Update the state first.
   * @param arr The returned marker array
   * @param link_names The list of link names for which the markers should be created.
   * @param color The color for the marker
   * @param ns The namespace for the markers
   * @param dur The rclcpp::Duration for which the markers should stay visible
   */
  void getRobotMarkers(
    visualization_msgs::msg::MarkerArray& arr,
    const std::vector<std::string>& link_names,
    const std_msgs::msg::ColorRGBA& color,
    const std::string& ns,
    const rclcpp::Duration& dur,
    bool include_attached = false)
  {
    updateCollisionBodyTransforms();
    static_cast<const RobotState*>(this)->getRobotMarkers(arr, link_names, color, ns, dur, include_attached);
  }

  /* Get a MarkerArray that fully describes the robot markers for a given robot.
   * @param arr The returned marker array
   * @param link_names The list of link names for which the markers should be created.
   */
  void getRobotMarkers(
    visualization_msgs::msg::MarkerArray& arr,
    const std::vector<std::string>& link_names,
    bool include_attached = false) const;

  /* Get a MarkerArray that fully describes the robot markers for a given robot. Update the state first.
   * @param arr The returned marker array
   * @param link_names The list of link names for which the markers should be created.
   */
  void getRobotMarkers(
    visualization_msgs::msg::MarkerArray& arr,
    const std::vector<std::string>& link_names,
    bool include_attached = false)
  {
    updateCollisionBodyTransforms();
    static_cast<const RobotState*>(this)->getRobotMarkers(arr, link_names, include_attached);
  }

  void printStatePositions(std::ostream& out = std::cout) const;

  /* Output to console the current state of the robot's joint limits */
  void printStatePositionsWithJointLimits(const JointModelGroup* jmg, std::ostream& out = std::cout) const;

  void printStateInfo(std::ostream& out = std::cout) const;

  void printTransforms(std::ostream& out = std::cout) const;

  void printTransform(const Eigen::Isometry3d& transform, std::ostream& out = std::cout) const;

  void printDirtyInfo(std::ostream& out = std::cout) const;

  std::string getStateTreeString() const;

  /**
   * @brief Transform pose from the robot model's base frame to the reference frame of the IK solver
   * @param pose - the input to change
   * @param solver - a kin solver whose base frame is important to us
   * @return true if no error
   */
  bool setToIKSolverFrame(Eigen::Isometry3d& pose, const KinematicsBaseConstPtr& solver);

  /**
   * @brief Transform pose from the robot model's base frame to the reference frame of the IK solver
   * @param pose - the input to change
   * @param ik_frame - the name of frame of reference of base of ik solver
   * @return true if no error
   */
  bool setToIKSolverFrame(Eigen::Isometry3d& pose, const std::string& ik_frame);

private:
  void allocMemory();
  void init();
  void copyFrom(const RobotState& other);

  void markDirtyJointTransforms(const JointModel* joint)
  {
    dirty_joint_transforms_[joint->getJointIndex()] = 1;
    dirty_link_transforms_ =
      dirty_link_transforms_ == nullptr ? joint : robot_model_->getCommonRoot(dirty_link_transforms_, joint);
  }

  void markDirtyJointTransforms(const JointModelGroup* group)
  {
    for (const JointModel* jm : group->getActiveJointModels())
      dirty_joint_transforms_[jm->getJointIndex()] = 1;
    dirty_link_transforms_ = dirty_link_transforms_ == nullptr ?
                               group->getCommonRoot() :
                               robot_model_->getCommonRoot(dirty_link_transforms_, group->getCommonRoot());
  }

  void markVelocity();
  void markAcceleration();
  void markEffort();

  void updateMimicJoint(const JointModel* joint);

  /* Update all mimic joints within group */
  void updateMimicJoints(const JointModelGroup* group);

  void updateLinkTransformsInternal(const JointModel* start);

  void
  getMissingKeys(const std::map<std::string, double>& variable_map, std::vector<std::string>& missing_variables) const;
  void getStateTreeJointString(std::ostream& ss, const JointModel* jm, const std::string& pfx0, bool last) const;

  /* This function is only called in debug mode */
  bool checkJointTransforms(const JointModel* joint) const;

  /* This function is only called in debug mode */
  bool checkLinkTransforms() const;

  /* This function is only called in debug mode */
  bool checkCollisionTransforms() const;

  /* Get the closest link in the kinematic tree to `frame`.
   *
   * Helper function for getRigidlyConnectedParentLinkModel,
   * which resolves attached objects / subframes.
   */
  const LinkModel* getLinkModelIncludingAttachedBodies(const std::string& frame) const;

  RobotModelConstPtr robot_model_;

  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_or_acceleration_;
  bool has_velocity_ = false;
  bool has_acceleration_ = false;
  bool has_effort_ = false;

  const JointModel* dirty_link_transforms_ = nullptr;
  const JointModel* dirty_collision_body_transforms_ = nullptr;

  // All the following transform variables point into aligned memory.
  // They are updated lazily, based on the flags in dirty_joint_transforms_
  // resp. the pointers dirty_link_transforms_ and dirty_collision_body_transforms_
  std::vector<Eigen::Isometry3d> variable_joint_transforms_;  ///< Local transforms of all joints
  std::vector<Eigen::Isometry3d> global_link_transforms_;  ///< Transforms from model frame to link frame for each link
  std::vector<Eigen::Isometry3d> global_collision_body_transforms_;  ///< Transforms from model frame to collision
                                                                     ///< bodies
  std::vector<unsigned char> dirty_joint_transforms_;

  /* All attached bodies that are part of this state, indexed by their name */
  std::map<std::string, std::unique_ptr<AttachedBody>> attached_body_map_;

  /* This event is called when there is a change in the attached bodies for this state;
   * The event specifies the body that changed and whether it was just attached or about to be detached. */
  AttachedBodyCallback attached_body_update_callback_;

  /* For certain operations a state needs a random number generator. However, it may be slightly expensive
   * to allocate the random number generator if many state instances are generated. For this reason, the generator
   * is allocated on a need basis, by the getRandomNumberGenerator() function. Never use the rng_ member directly, but
   * call getRandomNumberGenerator() instead. */
  std::unique_ptr<random_numbers::RandomNumberGenerator> rng_;
};

/* Operator overload for printing variable bounds to a stream */
std::ostream& operator<<(std::ostream& out, const RobotState& s);
}  // namespace tobas
