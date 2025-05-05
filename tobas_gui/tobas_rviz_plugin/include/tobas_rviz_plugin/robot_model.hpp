#pragma once

#include <iostream>

#include <srdfdom/model.h>
#include <eigen3/Eigen/Geometry>
#include <rclcpp/logging.hpp>

#include "../include/tobas_rviz_plugin/class_forward.hpp"
#include "../include/tobas_rviz_plugin/exceptions.hpp"
#include "../include/tobas_rviz_plugin/fixed_joint_model.hpp"
#include "../include/tobas_rviz_plugin/floating_joint_model.hpp"
#include "../include/tobas_rviz_plugin/joint_model_group.hpp"
#include "../include/tobas_rviz_plugin/planar_joint_model.hpp"
#include "../include/tobas_rviz_plugin/prismatic_joint_model.hpp"
#include "../include/tobas_rviz_plugin/revolute_joint_model.hpp"

namespace tobas
{
TOBAS_CLASS_FORWARD(RobotModel);  // Defines RobotModelPtr, ConstPtr, WeakPtr... etc

static inline void checkInterpolationParamBounds(const rclcpp::Logger& logger, double t)
{
  if (std::isnan(t) || std::isinf(t)) {
    throw Exception("Interpolation parameter is NaN or inf.");
  }

  RCLCPP_WARN_STREAM_EXPRESSION(logger, t < 0. || t > 1., "Interpolation parameter is not in the range [0, 1]: " << t);
}

/* Definition of a kinematic model. This class is not thread safe, however multiple instances can be created */
class RobotModel
{
public:
  /* Construct a kinematic model from a parsed description and a list of planning groups */
  RobotModel(const urdf::ModelInterfaceSharedPtr& urdf_model, const srdf::ModelConstSharedPtr& srdf_model);

  /* Destructor. Clear all memory. */
  ~RobotModel();

  /* Get the model name. */
  const std::string& getName() const
  {
    return model_name_;
  }

  /**
   * @brief Get the frame in which the transforms for this model are computed (when using a RobotState).
   * This frame depends on the root joint.
   * As such, the frame is either extracted from SRDF, or it is assumed to be the name of the root link.
   */
  const std::string& getModelFrame() const
  {
    return model_frame_;
  }

  /* Return true if the model is empty (has no root link, no joints) */
  bool isEmpty() const
  {
    return root_link_ == nullptr;
  }

  /* Get the parsed URDF model */
  const urdf::ModelInterfaceSharedPtr& getURDF() const
  {
    return urdf_;
  }

  /* Get the parsed SRDF model */
  const srdf::ModelConstSharedPtr& getSRDF() const
  {
    return srdf_;
  }

  /* Print information about the constructed model */
  void printModelInfo(std::ostream& out) const;

  /**
   * @brief Get the root joint.
   * There will be one root joint unless the model is empty.
   * This is either extracted from the SRDF, or a fixed joint is assumed, if no specification is given.
   */
  const JointModel* getRootJoint() const;

  /* Return the name of the root joint. Throws an exception if there is no root joint. */
  const std::string& getRootJointName() const
  {
    return getRootJoint()->getName();
  }

  /* Check if a joint exists. Return true if it does. */
  bool hasJointModel(const std::string& name) const;

  /* Get a joint by its name. Output error and return nullptr when the joint is missing. */
  const JointModel* getJointModel(const std::string& joint) const;

  /* Get a joint by its index. Output error and return nullptr when the link is missing. */
  const JointModel* getJointModel(size_t index) const;

  /* Get a joint by its name. Output error and return nullptr when the joint is missing. */
  JointModel* getJointModel(const std::string& joint);

  /* Get the array of joints, in the order they appear
      in the robot state. */
  const std::vector<const JointModel*>& getJointModels() const
  {
    return joint_model_vector_const_;
  }

  /**
   * @brief Get the array of joints, in the order they appear in the robot state.
   * This includes all types of joints (including mimic & fixed), as opposed to JointModelGroup::getJointModels().
   */
  const std::vector<JointModel*>& getJointModels()
  {
    return joint_model_vector_;
  }

  /* Get the array of joint names, in the order they appear in the robot state. */
  const std::vector<std::string>& getJointModelNames() const
  {
    return joint_model_names_vector_;
  }

  /* Get the array of joints that are active (not fixed, not mimic) in this model. */
  const std::vector<const JointModel*>& getActiveJointModels() const
  {
    return active_joint_model_vector_const_;
  }

  /* Get the array of active joint names, in the order they appear in the robot state. */
  const std::vector<std::string>& getActiveJointModelNames() const
  {
    return active_joint_model_names_vector_;
  }

  /* Get the array of joints that are active (not fixed, not mimic) in this model */
  const std::vector<JointModel*>& getActiveJointModels()
  {
    return active_joint_model_vector_;
  }

  /* This is a list of all single-dof joints (including mimic joints) */
  const std::vector<const JointModel*>& getSingleDOFJointModels() const
  {
    return single_dof_joints_;
  }

  /* This is a list of all multi-dof joints */
  const std::vector<const JointModel*>& getMultiDOFJointModels() const
  {
    return multi_dof_joints_;
  }

  /* Get the array of continuous joints, in the order they appear in the robot state. */
  const std::vector<const JointModel*>& getContinuousJointModels() const
  {
    return continuous_joint_model_vector_;
  }

  /* Get the array of mimic joints, in the order they appear in the robot state. */
  const std::vector<const JointModel*>& getMimicJointModels() const
  {
    return mimic_joints_;
  }

  const JointModel* getJointOfVariable(int variable_index) const
  {
    return joints_of_variable_[variable_index];
  }

  const JointModel* getJointOfVariable(const std::string& variable) const
  {
    return joints_of_variable_[getVariableIndex(variable)];
  }

  std::size_t getJointModelCount() const
  {
    return joint_model_vector_.size();
  }

  /* Get the physical root link of the robot. */
  const LinkModel* getRootLink() const;

  /* Get the name of the root link of the robot. */
  const std::string& getRootLinkName() const
  {
    return getRootLink()->getName();
  }

  /**
   * @brief Check if a link exists. Return true if it does.
   *
   * If this is followed by a call to getLinkModel(), better use the latter with the has_link argument.
   */
  bool hasLinkModel(const std::string& name) const;

  /* Get a link by its name. Output error and return nullptr when the link is missing. */
  const LinkModel* getLinkModel(const std::string& link, bool* has_link = nullptr) const;

  /* Get a link by its index. Output error and return nullptr when the link is missing. */
  const LinkModel* getLinkModel(size_t index) const;

  /* Get a link by its name. Output error and return nullptr when the link is missing. */
  LinkModel* getLinkModel(const std::string& link, bool* has_link = nullptr);

  /**
   * @brief Get the latest link upwards the kinematic tree, which is only connected via fixed joints
   *
   * If jmg is given, all links that are not active in this JMG are considered fixed.
   * Otherwise only fixed joints are considered fixed.
   *
   * This is useful, if the link should be warped to a specific pose using updateStateWithLinkAt().
   * As updateStateWithLinkAt() warps only the specified link and its descendants, you might not
   * achieve what you expect, if link is an abstract frame name. Considering the following example:
   * root -> arm0 -> ... -> armN -> wrist -- grasp_frame
   *                                      -- palm -> end effector ...
   * Calling updateStateWithLinkAt(grasp_frame), will not warp the end effector, which is probably
   * what you went for. Instead, updateStateWithLinkAt(getRigidlyConnectedParentLinkModel(grasp_frame), ...)
   * will actually warp wrist (and all its descendants).
   */
  static const LinkModel*
  getRigidlyConnectedParentLinkModel(const LinkModel* link, const JointModelGroup* jmg = nullptr);

  /* Get the array of links */
  const std::vector<const LinkModel*>& getLinkModels() const
  {
    return link_model_vector_const_;
  }

  /* Get the array of links */
  const std::vector<LinkModel*>& getLinkModels()
  {
    return link_model_vector_;
  }

  /* Get the link names (of all links) */
  const std::vector<std::string>& getLinkModelNames() const
  {
    return link_model_names_vector_;
  }

  /* Get the link models that have some collision geometry associated to themselves */
  const std::vector<const LinkModel*>& getLinkModelsWithCollisionGeometry() const
  {
    return link_models_with_collision_geometry_vector_;
  }

  /* Get the names of the link models that have some collision geometry associated to themselves */
  const std::vector<std::string>& getLinkModelNamesWithCollisionGeometry() const
  {
    return link_model_names_with_collision_geometry_vector_;
  }

  std::size_t getLinkModelCount() const
  {
    return link_model_vector_.size();
  }

  std::size_t getLinkGeometryCount() const
  {
    return link_geometry_count_;
  }

  /* Compute the random values for a RobotState */
  void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values) const;

  /* Compute the default values for a RobotState */
  void getVariableDefaultPositions(double* values) const;

  /* Compute the random values for a RobotState */
  void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, std::vector<double>& values) const
  {
    values.resize(variable_count_);
    getVariableRandomPositions(rng, &values[0]);
  }

  /* Compute the default values for a RobotState */
  void getVariableDefaultPositions(std::vector<double>& values) const
  {
    values.resize(variable_count_);
    getVariableDefaultPositions(&values[0]);
  }

  /* Compute the random values for a RobotState */
  void
  getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, std::map<std::string, double>& values) const;

  /* Compute the default values for a RobotState */
  void getVariableDefaultPositions(std::map<std::string, double>& values) const;

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

  /* Return the sum of joint distances between two states. An L1 norm. Only considers active joints. */
  double distance(const double* state1, const double* state2) const;

  /**
   * @brief Interpolate between "from" state, to "to" state. Mimic joints are correctly updated.
   *
   * @param from interpolate from this state
   * @param to to this state
   * @param t a fraction in the range [0 1]. If 1, the result matches "to" state exactly.
   * @param state holds the result
   */
  void interpolate(const double* from, const double* to, double t, double* state) const;

  /** \name Access to joint groups
   *  @{
   */

  /* Check if the JointModelGroup \e group exists */
  bool hasJointModelGroup(const std::string& group) const;

  /* Get a joint group from this model (by name) */
  const JointModelGroup* getJointModelGroup(const std::string& name) const;

  /* Get a joint group from this model (by name) */
  JointModelGroup* getJointModelGroup(const std::string& name);

  /* Get the available joint groups */
  const std::vector<const JointModelGroup*>& getJointModelGroups() const
  {
    return joint_model_groups_const_;
  }

  /* Get the available joint groups */
  const std::vector<JointModelGroup*>& getJointModelGroups()
  {
    return joint_model_groups_;
  }

  /* Get the names of all groups that are defined for this model */
  const std::vector<std::string>& getJointModelGroupNames() const
  {
    return joint_model_group_names_;
  }

  /* Check if an end effector exists */
  bool hasEndEffector(const std::string& eef) const;

  /* Get the joint group that corresponds to a given end-effector name */
  const JointModelGroup* getEndEffector(const std::string& name) const;

  /* Get the joint group that corresponds to a given end-effector name */
  JointModelGroup* getEndEffector(const std::string& name);

  /* Get the map between end effector names and the groups they correspond to */
  const std::vector<const JointModelGroup*>& getEndEffectors() const
  {
    return end_effectors_;
  }

  /* Get the number of variables that describe this model */
  std::size_t getVariableCount() const
  {
    return variable_count_;
  }

  /**
   * @brief Get the names of the variables that make up the joints that form this state.
   * Fixed joints have no DOF, so they are not here, but the variables for mimic joints are included.
   * The number of returned elements is always equal to getVariableCount().
   */
  const std::vector<std::string>& getVariableNames() const
  {
    return variable_names_;
  }

  /* Get the bounds for a specific variable. Throw an exception of variable is not found. */
  const VariableBounds& getVariableBounds(const std::string& variable) const
  {
    return getJointOfVariable(variable)->getVariableBounds(variable);
  }

  /* Get the bounds for all the active joints */
  const JointBoundsVector& getActiveJointModelsBounds() const
  {
    return active_joint_models_bounds_;
  }

  void
  getMissingVariableNames(const std::vector<std::string>& variables, std::vector<std::string>& missing_variables) const;

  /* Get the index of a variable in the robot state */
  size_t getVariableIndex(const std::string& variable) const;

  /* Get the deepest joint in the kinematic tree that is a common parent of both joints passed as argument */
  const JointModel* getCommonRoot(const JointModel* a, const JointModel* b) const
  {
    if (!a) {
      return b;
    }
    if (!b) {
      return a;
    }
    return joint_model_vector_[common_joint_roots_[a->getJointIndex() * joint_model_vector_.size() + b->getJointIndex()]];
  }

  /* A map of known kinematics solvers (associated to their group name) */
  void setKinematicsAllocators(const std::map<std::string, SolverAllocatorFn>& allocators);

protected:
  /* Get the transforms between link and all its rigidly attached descendants */
  void computeFixedTransforms(
    const LinkModel* link,
    const Eigen::Isometry3d& transform,
    LinkTransformMap& associated_transforms);

  /* Given two joints, find their common root */
  const JointModel* computeCommonRoot(const JointModel* a, const JointModel* b) const;

  /* Update the variable values for the state of a group with respect to the mimic joints. */
  void updateMimicJoints(double* values) const;

  // GENERIC INFO

  /* The name of the robot */
  std::string model_name_;

  /* The reference (base) frame for this model. The frame is either extracted from the SRDF as a virtual joint,
   * or it is assumed to be the name of the root link in the URDF */
  std::string model_frame_;

  srdf::ModelConstSharedPtr srdf_;

  urdf::ModelInterfaceSharedPtr urdf_;

  // LINKS

  /* The first physical link for the robot */
  const LinkModel* root_link_;

  /* A map from link names to their instances */
  LinkModelMap link_model_map_;

  /* The vector of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<LinkModel*> link_model_vector_;

  /* The vector of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<const LinkModel*> link_model_vector_const_;

  /* The vector of link names that corresponds to link_model_vector_ */
  std::vector<std::string> link_model_names_vector_;

  /* Only links that have collision geometry specified */
  std::vector<const LinkModel*> link_models_with_collision_geometry_vector_;

  /* The vector of link names that corresponds to link_models_with_collision_geometry_vector_ */
  std::vector<std::string> link_model_names_with_collision_geometry_vector_;

  /* Total number of geometric shapes in this model */
  std::size_t link_geometry_count_;

  // JOINTS

  /* The root joint */
  const JointModel* root_joint_;

  /* A map from joint names to their instances */
  JointModelMap joint_model_map_;

  /* The vector of joints in the model, in the order they appear in the state vector */
  std::vector<JointModel*> joint_model_vector_;

  /* The vector of joints in the model, in the order they appear in the state vector */
  std::vector<const JointModel*> joint_model_vector_const_;

  /* The vector of joint names that corresponds to joint_model_vector_ */
  std::vector<std::string> joint_model_names_vector_;

  /* The vector of joints in the model, in the order they appear in the state vector */
  std::vector<JointModel*> active_joint_model_vector_;

  /* The vector of joint names that corresponds to active_joint_model_vector_ */
  std::vector<std::string> active_joint_model_names_vector_;

  /* The vector of joints in the model, in the order they appear in the state vector */
  std::vector<const JointModel*> active_joint_model_vector_const_;

  /* The set of continuous joints this model contains */
  std::vector<const JointModel*> continuous_joint_model_vector_;

  /* The set of mimic joints this model contains */
  std::vector<const JointModel*> mimic_joints_;

  std::vector<const JointModel*> single_dof_joints_;

  std::vector<const JointModel*> multi_dof_joints_;

  /* For every two joints, the index of the common root for the joints is stored.
   *
   * for jointA, jointB the index of the common root is located in the array at location
   * jointA->getJointIndex() * nr.joints + jointB->getJointIndex().
   * The size of this array is nr.joints * nr.joints
   */
  std::vector<int> common_joint_roots_;

  // INDEXING

  /* The names of the DOF that make up this state (this is just a sequence of joint variable names; not
   * necessarily joint names!) */
  std::vector<std::string> variable_names_;

  /* Get the number of variables necessary to describe this model */
  std::size_t variable_count_;

  /* The state includes all the joint variables that make up the joints the state consists of.
   * This map gives the position in the state vector of the group for each of these variables.
   * Additionally, it includes the names of the joints and the index for the first variable of that joint. */
  VariableIndexMap joint_variables_index_map_;

  std::vector<int> active_joint_model_start_index_;

  /* The bounds for all the active joint models */
  JointBoundsVector active_joint_models_bounds_;

  /* The joints that correspond to each variable index */
  std::vector<const JointModel*> joints_of_variable_;

  // GROUPS

  /* A map from group names to joint groups */
  JointModelGroupMap joint_model_group_map_;

  /* The known end effectors */
  JointModelGroupMap end_effectors_map_;

  /* The array of joint model groups, in alphabetical order */
  std::vector<JointModelGroup*> joint_model_groups_;

  /* The array of joint model groups, in alphabetical order */
  std::vector<const JointModelGroup*> joint_model_groups_const_;

  /* A vector of all group names, in alphabetical order */
  std::vector<std::string> joint_model_group_names_;

  /* The array of end-effectors, in alphabetical order */
  std::vector<const JointModelGroup*> end_effectors_;

private:
  /* Given an URDF model and a SRDF model, build a full kinematic model */
  void buildModel(const urdf::ModelInterface& urdf_model, const srdf::Model& srdf_model);

  /* Given a SRDF model describing the groups, build up the groups in this kinematic model */
  void buildGroups(const srdf::Model& srdf_model);

  /* Compute helpful information about groups (that can be queried later) */
  void buildGroupsInfoSubgroups();

  /* Compute helpful information about groups (that can be queried later) */
  void buildGroupsInfoEndEffectors(const srdf::Model& srdf_model);

  /* Given the URDF model, build up the mimic joints (mutually constrained joints) */
  void buildMimic(const urdf::ModelInterface& urdf_model);

  /* Given a SRDF model describing the groups, build the default states defined in the SRDF */
  void buildGroupStates(const srdf::Model& srdf_model);

  /* Compute helpful information about joints */
  void buildJointInfo();

  /* For every joint, pre-compute the list of descendant joints & links */
  void computeDescendants();

  /* For every pair of joints, pre-compute the common roots of the joints */
  void computeCommonRoots();

  /* (This function is mostly intended for internal use). Given a parent link, build up (recursively),
   * the kinematic model by walking  down the tree*/
  JointModel* buildRecursive(LinkModel* parent, const urdf::Link* link, const srdf::Model& srdf_model);

  /* Construct a JointModelGroup given a SRDF description \e group */
  bool addJointModelGroup(const srdf::Model::Group& group);

  /* Given a child link and a srdf model, build up the corresponding JointModel object */
  JointModel* constructJointModel(const urdf::Link* child_link, const srdf::Model& srdf_model);

  /* Given a urdf link, build the corresponding LinkModel object */
  LinkModel* constructLinkModel(const urdf::Link* urdf_link);

  /* Given a geometry spec from the URDF and a filename (for a mesh), construct the corresponding shape object */
  shapes::ShapePtr constructShape(const urdf::Geometry* geom);
};
}  // namespace tobas
