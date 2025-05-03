#pragma once

#include <functional>
#include <set>
#include <string>
#include <srdfdom/model.h>

#include "./joint_model.hpp"
#include "./link_model.hpp"
#include "./kinematics_base.hpp"

namespace tobas
{
class RobotModel;
class JointModelGroup;

/* Function type that allocates a kinematics solver for a particular group */
typedef std::function<KinematicsBasePtr(const JointModelGroup*)> SolverAllocatorFn;

/* Map from group instances to allocator functions & bijections */
using SolverAllocatorMapFn = std::map<const JointModelGroup*, SolverAllocatorFn>;

/* Map of names to instances for JointModelGroup */
using JointModelGroupMap = std::map<std::string, JointModelGroup*>;

/* Map of names to const instances for JointModelGroup */
using JointModelGroupMapConst = std::map<std::string, const JointModelGroup*>;

using JointBoundsVector = std::vector<const JointModel::Bounds*>;

class JointModelGroup
{
public:
  struct KinematicsSolver
  {
    KinematicsSolver() : default_ik_timeout_(0.5)
    {
    }

    // Return a flag indicating whether the state of the solver is initialized
    operator bool() const
    {
      return allocator_ && !bijection_.empty() && solver_instance_;
    }

    void reset()
    {
      solver_instance_.reset();
      bijection_.clear();
    }

    // Function type that allocates a kinematics solver for a particular group
    SolverAllocatorFn allocator_;

    /**
     * The mapping between the order of the joints in the group and the order of the joints in the kinematicsbsolver.
     * An element bijection[i] at index \e i in this array, maps the variable at index bijection[i] in this group to the
     * variable at index i in the kinematic solver.
     */
    std::vector<size_t> bijection_;

    KinematicsBasePtr solver_instance_;

    double default_ik_timeout_;
  };

  // Map from group instances to allocator functions & bijections
  using KinematicsSolverMap = std::map<const JointModelGroup*, KinematicsSolver>;

  JointModelGroup(
    const std::string& name,
    const srdf::Model::Group& config,
    const std::vector<const JointModel*>& joint_vector,
    const RobotModel* parent_model);

  ~JointModelGroup();

  /* Get the kinematic model this group is part of */
  const RobotModel& getParentModel() const
  {
    return *parent_model_;
  }

  /* Get the name of the joint group */
  const std::string& getName() const
  {
    return name_;
  }

  /* get the SRDF configuration this group is based on */
  const srdf::Model::Group& getConfig() const
  {
    return config_;
  }

  /* Check if a joint is part of this group */
  bool hasJointModel(const std::string& joint) const;

  /* Check if a link is part of this group */
  bool hasLinkModel(const std::string& link) const;

  /* Get a joint by its name. Throw an exception if the joint is not part of this group. */
  const JointModel* getJointModel(const std::string& joint) const;

  /* Get a link by its name. Throw an exception if the link is not part of this group. */
  const LinkModel* getLinkModel(const std::string& link) const;

  /* Get all the joints in this group (including fixed and mimic joints). */
  const std::vector<const JointModel*>& getJointModels() const
  {
    return joint_model_vector_;
  }

  /* Get the names of the joints in this group. These are the names of the joints returned by getJointModels().
   */
  const std::vector<std::string>& getJointModelNames() const
  {
    return joint_model_name_vector_;
  }

  /* Get the active joints in this group (that  have controllable DOF). This does not include mimic joints. */
  const std::vector<const JointModel*>& getActiveJointModels() const
  {
    return active_joint_model_vector_;
  }

  /* Get the names of the active joints in this group. These are the names of the joints returned by getJointModels().
   */
  const std::vector<std::string>& getActiveJointModelNames() const
  {
    return active_joint_model_name_vector_;
  }

  /* Get the fixed joints that are part of this group */
  const std::vector<const JointModel*>& getFixedJointModels() const
  {
    return fixed_joints_;
  }

  /* Get the mimic joints that are part of this group */
  const std::vector<const JointModel*>& getMimicJointModels() const
  {
    return mimic_joints_;
  }

  /* Get the array of continuous joints used in this group (may include mimic joints). */
  const std::vector<const JointModel*>& getContinuousJointModels() const
  {
    return continuous_joint_model_vector_;
  }

  /**
   * @brief Get the names of the variables that make up the joints included in this group.
   * The number of returned elements is always equal to getVariableCount(). This includes mimic joints.
   */
  const std::vector<std::string>& getVariableNames() const
  {
    return variable_names_;
  }

  /**
   * @brief Unlike a complete kinematic model, a group may contain disconnected parts of the kinematic tree
   * -- a set of smaller trees. This function gives the roots of those smaller trees. Furthermore,
   * it is ensured that the roots are on different branches in the kinematic tree.
   * This means that in following any root in the given list, none of the other returned roots will be encountered.
   */
  const std::vector<const JointModel*>& getJointRoots() const
  {
    return joint_roots_;
  }

  /* Get the common root of all joint roots; not necessarily part of this group */
  const JointModel* getCommonRoot() const
  {
    return common_root_;
  }

  /* Get the links that are part of this joint group */
  const std::vector<const LinkModel*>& getLinkModels() const
  {
    return link_model_vector_;
  }

  /* Get the names of the links that are part of this joint group */
  const std::vector<std::string>& getLinkModelNames() const
  {
    return link_model_name_vector_;
  }

  /* Get the names of the links that are part of this joint group and also have geometry associated with them */
  const std::vector<std::string>& getLinkModelNamesWithCollisionGeometry() const
  {
    return link_model_with_geometry_name_vector_;
  }

  /**
   * @brief Get the names of the links that are to be updated when the state of this group changes.
   * This includes links that are in the kinematic model but outside this group,
   * if those links are descendants of joints in this group that have their values updated.
   * The order is the correct order for updating the corresponding states.
   */
  const std::vector<const LinkModel*>& getUpdatedLinkModels() const
  {
    return updated_link_model_vector_;
  }

  /* Return the same data as getUpdatedLinkModels() but as a set */
  const std::set<const LinkModel*>& getUpdatedLinkModelsSet() const
  {
    return updated_link_model_set_;
  }

  /* Get the names of the links returned by getUpdatedLinkModels() */
  const std::vector<std::string>& getUpdatedLinkModelNames() const
  {
    return updated_link_model_name_vector_;
  }

  /**
   * @brief Get the names of the links that are to be updated when the state of this group changes.
   * This includes links that are in the kinematic model but outside this group,
   * if those links are descendants of joints in this group that have their values updated.
   */
  const std::vector<const LinkModel*>& getUpdatedLinkModelsWithGeometry() const
  {
    return updated_link_model_with_geometry_vector_;
  }

  /* Return the same data as getUpdatedLinkModelsWithGeometry() but as a set */
  const std::set<const LinkModel*>& getUpdatedLinkModelsWithGeometrySet() const
  {
    return updated_link_model_with_geometry_set_;
  }

  /* Get the names of the links returned by getUpdatedLinkModels() */
  const std::vector<std::string>& getUpdatedLinkModelsWithGeometryNames() const
  {
    return updated_link_model_with_geometry_name_vector_;
  }

  /* Get the names of the links returned by getUpdatedLinkModels() */
  const std::set<std::string>& getUpdatedLinkModelsWithGeometryNamesSet() const
  {
    return updated_link_model_with_geometry_name_set_;
  }

  /**
   * @brief True if this name is in the set of links that are to be updated when the state of this group changes.
   * This includes links that are in the kinematic model but outside this group,
   * if those links are descendants of joints in this group that have their values updated.
   */
  bool isLinkUpdated(const std::string& name) const
  {
    return updated_link_model_name_set_.find(name) != updated_link_model_name_set_.end();
  }

  /* Get the index locations in the complete robot state for all the variables in this group */
  const std::vector<int>& getVariableIndexList() const
  {
    return variable_index_list_;
  }

  /* Get the index of a variable within the group. Return -1 on error. */
  int getVariableGroupIndex(const std::string& variable) const;

  /* Get the names of the known default states (as specified in the SRDF) */
  const std::vector<std::string>& getDefaultStateNames() const
  {
    return default_states_names_;
  }

  void addDefaultState(const std::string& name, const std::map<std::string, double>& default_state);

  /* Get the values that correspond to a named state as read from the URDF. Return false on failure. */
  bool getVariableDefaultPositions(const std::string& name, std::map<std::string, double>& values) const;

  /* Compute the default values for the joint group */
  void getVariableDefaultPositions(std::map<std::string, double>& values) const;

  /* Compute the default values for the joint group */
  void getVariableDefaultPositions(std::vector<double>& values) const
  {
    values.resize(variable_count_);
    getVariableDefaultPositions(&values[0]);
  }

  /* Compute the default values for the joint group */
  void getVariableDefaultPositions(double* values) const;

  /* Compute random values for the state of the joint group */
  void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values) const
  {
    getVariableRandomPositions(rng, values, active_joint_models_bounds_);
  }

  /* Compute random values for the state of the joint group */
  void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, std::vector<double>& values) const
  {
    values.resize(variable_count_);
    getVariableRandomPositions(rng, &values[0], active_joint_models_bounds_);
  }

  /* Compute random values for the state of the joint group */
  void getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng,
    double* values,
    const double* near,
    const double distance) const
  {
    getVariableRandomPositionsNearBy(rng, values, active_joint_models_bounds_, near, distance);
  }
  /* Compute random values for the state of the joint group */
  void getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng,
    std::vector<double>& values,
    const std::vector<double>& near,
    double distance) const
  {
    values.resize(variable_count_);
    getVariableRandomPositionsNearBy(rng, &values[0], active_joint_models_bounds_, &near[0], distance);
  }

  /* Compute random values for the state of the joint group */
  void getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng,
    std::vector<double>& values,
    const std::vector<double>& near,
    const std::map<JointModel::JointType, double>& distance_map) const
  {
    values.resize(variable_count_);
    getVariableRandomPositionsNearBy(rng, &values[0], active_joint_models_bounds_, &near[0], distance_map);
  }

  /* Compute random values for the state of the joint group */
  void getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng,
    double* values,
    const double* near,
    const std::vector<double>& distances) const
  {
    getVariableRandomPositionsNearBy(rng, values, active_joint_models_bounds_, near, distances);
  }
  /* Compute random values for the state of the joint group */
  void getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng,
    std::vector<double>& values,
    const std::vector<double>& near,
    const std::vector<double>& distances) const
  {
    values.resize(variable_count_);
    getVariableRandomPositionsNearBy(rng, &values[0], active_joint_models_bounds_, &near[0], distances);
  }

  void getVariableRandomPositions(
    random_numbers::RandomNumberGenerator& rng,
    double* values,
    const JointBoundsVector& active_joint_bounds) const;

  /* Compute random values for the state of the joint group */
  void getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng,
    double* values,
    const JointBoundsVector& active_joint_bounds,
    const double* near,
    const double distance) const;

  /* Compute random values for the state of the joint group */
  void getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng,
    double* values,
    const JointBoundsVector& active_joint_bounds,
    const double* near,
    const std::map<JointModel::JointType, double>& distance_map) const;

  /* Compute random values for the state of the joint group */
  void getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng,
    double* values,
    const JointBoundsVector& active_joint_bounds,
    const double* near,
    const std::vector<double>& distances) const;

  bool enforcePositionBounds(double* state) const
  {
    return enforcePositionBounds(state, active_joint_models_bounds_);
  }

  bool enforcePositionBounds(double* state, const JointBoundsVector& active_joint_bounds) const;
  bool satisfiesPositionBounds(const double* state, double margin = 0.) const
  {
    return satisfiesPositionBounds(state, active_joint_models_bounds_, margin);
  }
  bool
  satisfiesPositionBounds(const double* state, const JointBoundsVector& active_joint_bounds, double margin = 0.) const;

  double getMaximumExtent() const
  {
    return getMaximumExtent(active_joint_models_bounds_);
  }
  double getMaximumExtent(const JointBoundsVector& active_joint_bounds) const;

  double distance(const double* state1, const double* state2) const;
  void interpolate(const double* from, const double* to, double t, double* state) const;

  /**
   * @brief Get the number of variables that describe this joint group.
   * This includes variables necessary for mimic joints, so will always be >= getActiveVariableCount()
   */
  unsigned int getVariableCount() const
  {
    return variable_count_;
  }

  /**
   * @brief Get the number of variables that describe the active joints in this joint group.
   *  This excludes variables necessary for mimic joints.
   */
  unsigned int getActiveVariableCount() const
  {
    return active_variable_count_;
  }

  /* Set the names of the subgroups for this group */
  void setSubgroupNames(const std::vector<std::string>& subgroups);

  /* Get the names of the groups that are subsets of this one (in terms of joints set) */
  const std::vector<std::string>& getSubgroupNames() const
  {
    return subgroup_names_;
  }

  /* Get the groups that are subsets of this one (in terms of joints set) */
  void getSubgroups(std::vector<const JointModelGroup*>& sub_groups) const;

  /* Check if the joints of group \e group are a subset of the joints in this group */
  bool isSubgroup(const std::string& group) const
  {
    return subgroup_names_set_.find(group) != subgroup_names_set_.end();
  }

  /* Check if this group is a linear chain */
  bool isChain() const
  {
    return is_chain_;
  }

  /* Return true if the group consists only of joints that are single DOF */
  bool isSingleDOFJoints() const
  {
    return is_single_dof_;
  }

  /* Check if this group was designated as an end-effector in the SRDF */
  bool isEndEffector() const
  {
    return !end_effector_name_.empty();
  }

  bool isContiguousWithinState() const
  {
    return is_contiguous_index_list_;
  }

  /* Return the name of the end effector, if this group is an end-effector */
  const std::string& getEndEffectorName() const
  {
    return end_effector_name_;
  }

  /* Set the name of the end-effector, and remember this group is indeed an end-effector. */
  void setEndEffectorName(const std::string& name);

  /**
   * @brief Set the End Effector Parent objectIf this group is an end-effector, specify the parent group
   * (e.g., the arm holding the eef) and the link the end effector connects to.
   */
  void setEndEffectorParent(const std::string& group, const std::string& link);

  /* Notify this group that there is an end-effector attached to it */
  void attachEndEffector(const std::string& eef_name);

  /* Get the name of the group this end-effector attaches to (first) and the name of the link in that group (second). */
  const std::pair<std::string, std::string>& getEndEffectorParentGroup() const
  {
    return end_effector_parent_;
  }

  /* Get the names of the end effectors attached to this group */
  const std::vector<std::string>& getAttachedEndEffectorNames() const
  {
    return attached_end_effector_names_;
  }

  /**
   * @brief Get the unique set of end effector tips included in a particular joint model group
   * as defined by the SRDF end effector elements.
   * e.g. for a humanoid robot this would return 4 tips for the hands and feet
   * @param tips - the output vector of link models of the tips
   * @return true on success
   */
  bool getEndEffectorTips(std::vector<const LinkModel*>& tips) const;

  /**
   * @brief Get the unique set of end effector tips included in a particular joint model group
   * as defined by the SRDF end effector elements.
   * e.g. for a humanoid robot this would return 4 tips for the hands and feet
   * @param tips - the output vector of link names of the tips
   * @return true on success
   */
  bool getEndEffectorTips(std::vector<std::string>& tips) const;

  /**
   * @brief Get one end effector tip, throwing an error if there ends up being more in the joint model group
   * This is a useful helper function because most planning groups (almost all) only have one tip
   * @return pointer to LinkModel, or nullptr on failure
   */
  const LinkModel* getOnlyOneEndEffectorTip() const;

  /* Get the bounds for all the active joints */
  const JointBoundsVector& getActiveJointModelsBounds() const
  {
    return active_joint_models_bounds_;
  }

  const std::pair<KinematicsSolver, KinematicsSolverMap>& getGroupKinematics() const
  {
    return group_kinematics_;
  }

  void
  setSolverAllocators(const SolverAllocatorFn& solver, const SolverAllocatorMapFn& solver_map = SolverAllocatorMapFn())
  {
    setSolverAllocators(std::make_pair(solver, solver_map));
  }

  void setSolverAllocators(const std::pair<SolverAllocatorFn, SolverAllocatorMapFn>& solvers);

  const KinematicsBaseConstPtr getSolverInstance() const
  {
    return group_kinematics_.first.solver_instance_;
  }

  const KinematicsBasePtr& getSolverInstance()
  {
    return group_kinematics_.first.solver_instance_;
  }

  bool canSetStateFromIK(const std::string& tip) const;

  bool setRedundantJoints(const std::vector<std::string>& joints)
  {
    if (group_kinematics_.first.solver_instance_) {
      return group_kinematics_.first.solver_instance_->setRedundantJoints(joints);
    }
    return false;
  }

  /* Get the default IK timeout */
  double getDefaultIKTimeout() const
  {
    return group_kinematics_.first.default_ik_timeout_;
  }

  /* Set the default IK timeout */
  void setDefaultIKTimeout(double ik_timeout);

  /**
   * @brief Return the mapping between the order of the joints in this group
   * and the order of the joints in the kinematics solver.
   * An element bijection[i] at index \e i in this array, maps the variable at index bijection[i] in this group to
   * the variable at index i in the kinematic solver.
   */
  const std::vector<size_t>& getKinematicsSolverJointBijection() const
  {
    return group_kinematics_.first.bijection_;
  }

  /* Print information about the constructed model */
  void printGroupInfo(std::ostream& out = std::cout) const;

  /* Check that the time to move between two waypoints is sufficient given velocity limits */
  bool isValidVelocityMove(
    const std::vector<double>& from_joint_pose,
    const std::vector<double>& to_joint_pose,
    double dt) const;

  /* Check that the time to move between two waypoints is sufficient given velocity limits */
  bool isValidVelocityMove(const double* from_joint_pose, const double* to_joint_pose, std::size_t array_size, double dt)
    const;

  /* Computes the indices of joint variables given a vector of joint names to look up */
  bool
  computeJointVariableIndices(const std::vector<std::string>& joint_names, std::vector<size_t>& joint_bijection) const;

  /**
   * @brief Get the lower and upper position limits of all active variables in the group.
   * @details In the case of variable without position bounds (e.g. continuous joints), the lower and upper limits are
   * set to infinity.
   * @return std::pair<Eigen::VectorXd, Eigen::VectorXd> Containing the lower and upper joint limits for all active
   * variables.
   */
  [[nodiscard]] std::pair<Eigen::VectorXd, Eigen::VectorXd> getLowerAndUpperLimits() const;

  /**
   * @brief Gets the pair of maximum joint velocities/accelerations for a given group. Asserts that the group contains
   * only single-variable joints,
   * @details In case of asymmetric velocity or acceleration limits, this function will return the most limiting
   * component.
   * @return std::pair<Eigen::VectorXd, Eigen::VectorXd> Containing the velocity and acceleration limits
   */
  [[nodiscard]] std::pair<Eigen::VectorXd, Eigen::VectorXd> getMaxVelocitiesAndAccelerationBounds() const;

protected:
  /**
   * @brief Update the variable values for the state of a group with respect to the mimic joints. This only updates
   * mimic joints that have the parent in this group. If there is a joint mimicking one that is outside the group,
   * there are no values to be read (\e values is only the group state).
   */
  void updateMimicJoints(double* values) const;

  /* Owner model */
  const RobotModel* parent_model_;

  /* Name of group */
  std::string name_;

  /* Joint instances in the order they appear in the group state */
  std::vector<const JointModel*> joint_model_vector_;

  /* Names of joints in the order they appear in the group state */
  std::vector<std::string> joint_model_name_vector_;

  /* Active joint instances in the order they appear in the group state */
  std::vector<const JointModel*> active_joint_model_vector_;

  /* Names of active joints in the order they appear in the group state */
  std::vector<std::string> active_joint_model_name_vector_;

  /* The joints that have no DOF (fixed) */
  std::vector<const JointModel*> fixed_joints_;

  /* Joints that mimic other joints */
  std::vector<const JointModel*> mimic_joints_;

  /* The set of continuous joints this group contains */
  std::vector<const JointModel*> continuous_joint_model_vector_;

  /* The names of the DOF that make up this group (this is just a sequence of joint variable names; not necessarily
   * joint names!) */
  std::vector<std::string> variable_names_;

  /* The names of the DOF that make up this group (this is just a sequence of joint variable names; not necessarily
   * joint names!) */
  std::set<std::string> variable_names_set_;

  /* A map from joint names to their instances. This includes all joints in the group. */
  JointModelMapConst joint_model_map_;

  /* The list of active joint models that are roots in this group */
  std::vector<const JointModel*> joint_roots_;

  /* The joint that is a common root for all joints in this group (not necessarily part of this group) */
  const JointModel* common_root_;

  /* The group includes all the joint variables that make up the joints the group consists of.
   * This map gives the position in the state vector of the group for each of these variables.
   * Additionally, it includes the names of the joints and the index for the first variable of that joint. */
  VariableIndexMap joint_variables_index_map_;

  /* The bounds for all the active joint models */
  JointBoundsVector active_joint_models_bounds_;

  /* The list of index values this group includes, with respect to a full robot state; this includes mimic joints. */
  std::vector<int> variable_index_list_;

  /* For each active joint model in this group, hold the index at which the corresponding joint state starts in the
   * group state */
  std::vector<int> active_joint_model_start_index_;

  /* The links that are on the direct lineage between joints
   * and joint_roots_, as well as the children of the joint leafs.
   * May not be in any particular order */
  std::vector<const LinkModel*> link_model_vector_;

  /* A map from link names to their instances */
  LinkModelMapConst link_model_map_;

  /* The names of the links in this group */
  std::vector<std::string> link_model_name_vector_;

  std::vector<const LinkModel*> link_model_with_geometry_vector_;

  /* The names of the links in this group that also have geometry */
  std::vector<std::string> link_model_with_geometry_name_vector_;

  /* The list of downstream link models in the order they should be updated
   * (may include links that are not in this group) */
  std::vector<const LinkModel*> updated_link_model_vector_;

  /* The list of downstream link models in the order they should be updated
   * (may include links that are not in this group) */
  std::set<const LinkModel*> updated_link_model_set_;

  /* The list of downstream link names in the order they should be updated
   * (may include links that are not in this group) */
  std::vector<std::string> updated_link_model_name_vector_;

  /* The list of downstream link names in the order they should be updated
   * (may include links that are not in this group) */
  std::set<std::string> updated_link_model_name_set_;

  /* The list of downstream link models in the order they should be updated
   * (may include links that are not in this group) */
  std::vector<const LinkModel*> updated_link_model_with_geometry_vector_;

  /* The list of downstream link models in the order they should be updated
   * (may include links that are not in this group) */
  std::set<const LinkModel*> updated_link_model_with_geometry_set_;

  /* The list of downstream link names in the order they should be updated
   * (may include links that are not in this group) */
  std::vector<std::string> updated_link_model_with_geometry_name_vector_;

  /* The list of downstream link names in the order they should be updated
   * (may include links that are not in this group) */
  std::set<std::string> updated_link_model_with_geometry_name_set_;

  /* The number of variables necessary to describe this group of joints */
  unsigned int variable_count_;

  /* The number of variables necessary to describe the active joints in this group of joints */
  unsigned int active_variable_count_;

  /* True if the state of this group is contiguous within the full robot state; this also means that
   * the index values in variable_index_list_ are consecutive integers */
  bool is_contiguous_index_list_;

  /* The set of labelled subgroups that are included in this group */
  std::vector<std::string> subgroup_names_;

  /* The set of labelled subgroups that are included in this group */
  std::set<std::string> subgroup_names_set_;

  /* If an end-effector is attached to this group, the name of that end-effector is stored in this variable */
  std::vector<std::string> attached_end_effector_names_;

  /* First: name of the group that is parent to this end-effector group; Second: the link this in the parent group that
   * this group attaches to */
  std::pair<std::string, std::string> end_effector_parent_;

  /* The name of the end effector, if this group is an end-effector */
  std::string end_effector_name_;

  bool is_chain_;

  bool is_single_dof_;

  struct GroupMimicUpdate
  {
    GroupMimicUpdate(int s, int d, double f, double o) : src(s), dest(d), factor(f), offset(o)
    {
    }
    int src;
    int dest;
    double factor;
    double offset;
  };

  std::vector<GroupMimicUpdate> group_mimic_update_;

  std::pair<KinematicsSolver, KinematicsSolverMap> group_kinematics_;

  srdf::Model::Group config_;

  /* The set of default states specified for this group in the SRDF */
  std::map<std::string, std::map<std::string, double> > default_states_;

  /* The names of the default states specified for this group in the SRDF */
  std::vector<std::string> default_states_names_;
};
}  // namespace tobas
