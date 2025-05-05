#pragma once

#include <string>
#include <vector>
#include <map>
#include <iostream>

#include <random_numbers/random_numbers.h>
#include <eigen3/Eigen/Geometry>

#include <tobas_visualization_msgs/msg/joint_limits.hpp>

#undef near

namespace tobas
{
struct VariableBounds
{
  VariableBounds()
    : min_position_(0.)
    , max_position_(0.)
    , position_bounded_(false)
    , min_velocity_(0.)
    , max_velocity_(0.)
    , velocity_bounded_(false)
    , min_acceleration_(0.)
    , max_acceleration_(0.)
    , acceleration_bounded_(false)
    , min_jerk_(0.)
    , max_jerk_(0.)
    , jerk_bounded_(false)
  {
  }

  double min_position_;
  double max_position_;
  bool position_bounded_;

  double min_velocity_;
  double max_velocity_;
  bool velocity_bounded_;

  double min_acceleration_;
  double max_acceleration_;
  bool acceleration_bounded_;

  double min_jerk_;
  double max_jerk_;
  bool jerk_bounded_;
};

class LinkModel;
class JointModel;

/* Data type for holding mappings from variable names to their position in a state vector */
typedef std::map<std::string, size_t> VariableIndexMap;

/* Data type for holding mappings from variable names to their bounds */
using VariableBoundsMap = std::map<std::string, VariableBounds>;

/* Map of names to instances for JointModel */
using JointModelMap = std::map<std::string, JointModel*>;

/* Map of names to const instances for JointModel */
using JointModelMapConst = std::map<std::string, const JointModel*>;

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
    FIXED
  };

  /* The datatype for the joint bounds */
  using Bounds = std::vector<VariableBounds>;

  /**
   * @brief Constructs a joint named \e name
   *
   * @param name                   The joint name
   * @param index                  The index of the joint in the RobotModel
   * @param first_variable_index   The index of the first variable in the RobotModel
   */
  JointModel(const std::string& name, size_t joint_index, size_t first_variable_index);

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

  /* Get the type of joint as a string */
  std::string getTypeName() const;

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

  /**
   * @brief Get the local names of the variable that make up the joint
   * (suffixes that are attached to joint names to construct the variable names).
   * For single DOF joints, this will be empty.
   */
  const std::vector<std::string>& getLocalVariableNames() const
  {
    return local_variable_names_;
  }

  /* Check if a particular variable is known to this joint */
  bool hasVariable(const std::string& variable) const
  {
    return variable_index_map_.find(variable) != variable_index_map_.end();
  }

  /* Get the number of variables that describe this joint */
  std::size_t getVariableCount() const
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
   * @brief Provide a default value for the joint given the default joint variable bounds (maintained internally).
   * Most joints will use the default implementation provided in this base class,
   * but the quaternion for example needs a different implementation.
   * Enough memory is assumed to be allocated.
   */
  void getVariableDefaultPositions(double* values) const
  {
    getVariableDefaultPositions(values, variable_bounds_);
  }

  /**
   * @brief Provide a default value for the joint given the joint variable bounds.
   * Most joints will use the default implementation provided in this base class,
   * but the quaternion for example needs a different implementation.
   * Enough memory is assumed to be allocated.
   */
  virtual void getVariableDefaultPositions(double* values, const Bounds& other_bounds) const = 0;

  /**
   * @brief Provide random values for the joint variables (within default bounds).
   * Enough memory is assumed to be allocated.
   */
  void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values) const
  {
    getVariableRandomPositions(rng, values, variable_bounds_);
  }

  /**
   * @brief Provide random values for the joint variables (within specified bounds).
   * Enough memory is assumed to be allocated.
   */
  virtual void getVariableRandomPositions(
    random_numbers::RandomNumberGenerator& rng,
    double* values,
    const Bounds& other_bounds) const = 0;

  /**
   * @brief Provide random values for the joint variables (within default bounds).
   * Enough memory is assumed to be allocated.
   */
  void getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng,
    double* values,
    const double* near,
    const double distance) const
  {
    getVariableRandomPositionsNearBy(rng, values, variable_bounds_, near, distance);
  }

  /**
   * @brief Provide random values for the joint variables (within specified bounds).
   * Enough memory is assumed to be allocated.
   */
  virtual void getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng,
    double* values,
    const Bounds& other_bounds,
    const double* near,
    const double distance) const = 0;

  /* Check if the set of values for the variables of this joint are within bounds. */
  bool satisfiesPositionBounds(const double* values, double margin = 0.) const
  {
    return satisfiesPositionBounds(values, variable_bounds_, margin);
  }

  /* Check if the set of position values for the variables of this joint are within bounds, up to some margin. */
  virtual bool satisfiesPositionBounds(const double* values, const Bounds& other_bounds, double margin) const = 0;

  /**
   * @brief Force the specified values to be inside bounds and normalized.
   * Quaternions are normalized, continuous revolute joints are made between -Pi and Pi.
   * Returns true if changes were made.
   */
  bool enforcePositionBounds(double* values) const
  {
    return enforcePositionBounds(values, variable_bounds_);
  }

  /**
   * @brief Force the specified values to be inside bounds and normalized.
   * Quaternions are normalized, continuous revolute joints are made between -Pi and Pi.
   * Return true if changes were made.
   */
  virtual bool enforcePositionBounds(double* values, const Bounds& other_bounds) const = 0;

  /**
   * @brief Harmonize position of revolute joints, adding/subtracting multiples of 2*Pi to bring them back into bounds.
   * Return true if changes were made.
   */
  virtual bool harmonizePosition(double* values, const Bounds& other_bounds) const;
  bool harmonizePosition(double* values) const
  {
    return harmonizePosition(values, variable_bounds_);
  }

  /* Check if the set of velocities for the variables of this joint are within bounds. */
  bool satisfiesVelocityBounds(const double* values, double margin = 0.) const
  {
    return satisfiesVelocityBounds(values, variable_bounds_, margin);
  }

  /* Check if the set of velocities for the variables of this joint are within bounds, up to some margin. */
  virtual bool satisfiesVelocityBounds(const double* values, const Bounds& other_bounds, double margin) const;

  /* Force the specified velocities to be within bounds. Return true if changes were made. */
  bool enforceVelocityBounds(double* values) const
  {
    return enforceVelocityBounds(values, variable_bounds_);
  }

  /* Force the specified velocities to be inside bounds. Return true if changes were made. */
  virtual bool enforceVelocityBounds(double* values, const Bounds& other_bounds) const;

  /* Check if the set of accelerations for the variables of this joint are within bounds. */
  bool satisfiesAccelerationBounds(const double* values, double margin = 0.) const
  {
    return satisfiesAccelerationBounds(values, variable_bounds_, margin);
  }

  /* Check if the set of accelerations for the variables of this joint are within bounds, up to some margin. */
  virtual bool satisfiesAccelerationBounds(const double* values, const Bounds& other_bounds, double margin) const;

  /* Check if the set of jerks for the variables of this joint are within bounds. */
  bool satisfiesJerkBounds(const double* values, double margin = 0.) const
  {
    return satisfiesJerkBounds(values, variable_bounds_, margin);
  }

  /* Check if the set of jerks for the variables of this joint are within bounds, up to some margin. */
  virtual bool satisfiesJerkBounds(const double* values, const Bounds& other_bounds, double margin) const;

  /* Get the bounds for a variable. Throw an exception if the variable was not found */
  const VariableBounds& getVariableBounds(const std::string& variable) const;

  /* Get the variable bounds for this joint, in the same order as the names returned by getVariableNames() */
  const Bounds& getVariableBounds() const
  {
    return variable_bounds_;
  }

  /* Set the lower and upper bounds for a variable. Throw an exception if the variable was not found. */
  void setVariableBounds(const std::string& variable, const VariableBounds& bounds);

  /* Override joint limits loaded from URDF. Unknown variables are ignored. */
  void setVariableBounds(const std::vector<tobas_visualization_msgs::msg::JointLimits>& jlim);

  /* Get the joint limits known to this model, as a message. */
  const std::vector<tobas_visualization_msgs::msg::JointLimits>& getVariableBoundsMsg() const
  {
    return variable_bounds_msg_;
  }

  /* Compute the distance between two joint states of the same model (represented by the variable values) */
  virtual double distance(const double* value1, const double* value2) const = 0;

  /**
   * @brief Get the factor that should be applied to the value returned by distance()
   * when that value is used in compound distances
   */
  double getDistanceFactor() const
  {
    return distance_factor_;
  }

  /**
   * @brief Set the factor that should be applied to the value returned by distance()
   * when that value is used in compound distances
   */
  void setDistanceFactor(double factor)
  {
    distance_factor_ = factor;
  }

  /* Get the dimension of the state space that corresponds to this joint */
  virtual unsigned int getStateSpaceDimension() const = 0;

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

  /* Check if this joint is passive */
  bool isPassive() const
  {
    return passive_;
  }

  void setPassive(bool flag)
  {
    passive_ = flag;
  }

  /**
   * @brief Computes the state that lies at time t in [0, 1] on the segment that connects from state to to state.
   * The memory location of state is not required to be different from the memory of either from or to.
   */
  virtual void interpolate(const double* from, const double* to, const double t, double* state) const = 0;

  /* Get the extent of the state space (the maximum value distance() can ever report) */
  virtual double getMaximumExtent(const Bounds& other_bounds) const = 0;

  double getMaximumExtent() const
  {
    return getMaximumExtent(variable_bounds_);
  }

  /**
   * @brief Given the joint values for a joint, compute the corresponding transform.
   * The computed transform is guaranteed to be a valid isometry.
   */
  virtual void computeTransform(const double* joint_values, Eigen::Isometry3d& transf) const = 0;

  /**
   * @brief Given the transform generated by joint, compute the corresponding joint values.
   * Make sure the passed transform is a valid isometry.
   */
  virtual void computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const = 0;

private:
  /* Name of the joint */
  std::string name_;

  /* Index for this joint in the array of joints of the complete model */
  size_t joint_index_;

  /* The index of this joint's first variable, in the complete robot state */
  size_t first_variable_index_;

protected:
  void computeVariableBoundsMsg();

  /* The type of joint */
  JointType type_;

  /* The local names to use for the variables that make up this joint */
  std::vector<std::string> local_variable_names_;

  /* The full names to use for the variables that make up this joint */
  std::vector<std::string> variable_names_;

  /* The bounds for each variable (low, high) in the same order as variable_names_ */
  Bounds variable_bounds_;

  std::vector<tobas_visualization_msgs::msg::JointLimits> variable_bounds_msg_;

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

  /* Specify whether this joint is marked as passive in the SRDF */
  bool passive_;

  /* The factor applied to the distance between two joint states */
  double distance_factor_;
};

/* Operator overload for printing variable bounds to a stream */
std::ostream& operator<<(std::ostream& out, const VariableBounds& b);
}  // namespace tobas
