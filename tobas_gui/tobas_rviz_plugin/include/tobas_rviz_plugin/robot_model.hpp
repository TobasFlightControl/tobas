// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <urdf/model.h>
#include <eigen3/Eigen/Geometry>

#include "./joint_model/fixed_joint_model.hpp"
#include "./joint_model/floating_joint_model.hpp"
#include "./joint_model/planar_joint_model.hpp"
#include "./joint_model/prismatic_joint_model.hpp"
#include "./joint_model/revolute_joint_model.hpp"
#include "./link_model.hpp"

namespace tobas
{
/* Definition of a kinematic model. This class is not thread safe, however multiple instances can be created. */
class RobotModel
{
public:
  using SharedPtr = std::shared_ptr<RobotModel>;
  using ConstSharedPtr = std::shared_ptr<const RobotModel>;

  explicit RobotModel(const urdf::ModelInterfaceSharedPtr& urdf_model);
  ~RobotModel();

  /* Get the frame in which the transforms for this model are computed (when using a RobotState). */
  const std::string& getModelFrame() const;

  /* Get the parsed URDF model. */
  const urdf::ModelInterfaceSharedPtr& getURDF() const;

  /* Get the root joint. */
  const JointModel* getRootJoint() const;

  /* Check if a joint exists. Return true if it does. */
  bool hasJointModel(const std::string& name) const;

  /* Get a joint by its name. Output error and return nullptr when the joint is missing. */
  const JointModel* getJointModel(const std::string& joint) const;

  /* Get a joint by its index. Output error and return nullptr when the link is missing. */
  const JointModel* getJointModel(size_t index) const;

  /* Get a joint by its name. Output error and return nullptr when the joint is missing. */
  JointModel* getJointModel(const std::string& joint);

  /* This is a list of all single-dof joints (including mimic joints). */
  const std::vector<const JointModel*>& getSingleDOFJointModels() const;

  /* This is a list of all multi-dof joints. */
  const std::vector<const JointModel*>& getMultiDOFJointModels() const;

  const JointModel* getJointOfVariable(int variable_index) const;
  const JointModel* getJointOfVariable(const std::string& variable) const;

  size_t getJointModelCount() const;

  /* Get the physical root link of the robot. */
  const LinkModel* getRootLink() const;

  /* Get a link by its name. Output error and return nullptr when the link is missing. */
  const LinkModel* getLinkModel(const std::string& link, bool* has_link = nullptr) const;

  /* Get a link by its index. Output error and return nullptr when the link is missing. */
  const LinkModel* getLinkModel(size_t index) const;

  /* Get a link by its name. Output error and return nullptr when the link is missing. */
  LinkModel* getLinkModel(const std::string& link, bool* has_link = nullptr);

  size_t getLinkModelCount() const;

  size_t getLinkGeometryCount() const;

  /* Compute the default values for a \e RobotState. */
  void getVariableDefaultPositions(double* values) const;
  void getVariableDefaultPositions(std::vector<double>& values) const;
  void getVariableDefaultPositions(std::map<std::string, double>& values) const;

  /* Get the number of variables that describe this model. */
  size_t getVariableCount() const;

  /* Get the names of the variables that make up the joints that form this state. */
  const std::vector<std::string>& getVariableNames() const;

  /* Get the index of a variable in the robot state */
  size_t getVariableIndex(const std::string& variable) const;

  /* Get the deepest joint in the kinematic tree that is a common parent of both joints passed as argument. */
  const JointModel* getCommonRoot(const JointModel* a, const JointModel* b) const;

protected:
  /* Get the transforms between link and all its rigidly attached descendants. */
  void computeFixedTransforms(
    const LinkModel* link,
    const Eigen::Isometry3d& transform,
    LinkTransformMap& associated_transforms);

  /* Update the variable values for the state of a group with respect to the mimic joints. */
  void updateMimicJoints(double* values) const;

  // GENERIC INFO

  /* The name of the robot */
  std::string model_name_;

  /* The reference (base) frame for this model. */
  std::string model_frame_;

  urdf::ModelInterfaceSharedPtr urdf_;

  // LINKS

  /* The first physical link for the robot */
  const LinkModel* root_link_;

  /* A map from link names to their instances */
  std::map<std::string, LinkModel*> link_model_map_;

  /* The vector of links that are updated when \e computeTransforms() is called, in the order they are updated */
  std::vector<LinkModel*> link_model_vector_;

  /* The vector of links that are updated when \e computeTransforms() is called, in the order they are updated */
  std::vector<const LinkModel*> link_model_vector_const_;

  /* The vector of link names that corresponds to link_model_vector_ */
  std::vector<std::string> link_model_names_vector_;

  /* Only links that have collision geometry specified */
  std::vector<const LinkModel*> link_models_with_collision_geometry_vector_;

  /* The vector of link names that corresponds to \e link_models_with_collision_geometry_vector_ */
  std::vector<std::string> link_model_names_with_collision_geometry_vector_;

  /* Total number of geometric shapes in this model */
  size_t link_geometry_count_;

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
   * For jointA, jointB the index of the common root is located in the array at location
   * jointA->getJointIndex() * nr.joints + jointB->getJointIndex().
   * The size of this array is nr.joints * nr.joints. */
  std::vector<int> common_joint_roots_;

  // INDEXING

  /* The names of the DOF that make up this state
   * (This is just a sequence of joint variable names; not necessarily joint names!) */
  std::vector<std::string> variable_names_;

  /* Get the number of variables necessary to describe this model */
  size_t variable_count_;

  /* The state includes all the joint variables that make up the joints the state consists of.
   * This map gives the position in the state vector of the group for each of these variables.
   * Additionally, it includes the names of the joints and the index for the first variable of that joint. */
  VariableIndexMap joint_variables_index_map_;

  std::vector<int> active_joint_model_start_index_;

  /* The joints that correspond to each variable index */
  std::vector<const JointModel*> joints_of_variable_;

  /* A vector of all group names, in alphabetical order */
  std::vector<std::string> joint_model_group_names_;

private:
  /* Given an URDF model, build a full kinematic model. */
  void buildModel(const urdf::ModelInterface& urdf_model);

  /* Given the URDF model, build up the mimic joints (mutually constrained joints). */
  void buildMimic(const urdf::ModelInterface& urdf_model);

  /* Compute helpful information about joints. */
  void buildJointInfo();

  /* For every joint, pre-compute the list of descendant joints & links. */
  void computeDescendants();

  /* For every pair of joints, pre-compute the common roots of the joints. */
  void computeCommonRoots();

  /* Given a parent link, build up (recursively), the kinematic model by walking  down the tree. */
  JointModel* buildRecursive(LinkModel* parent, const urdf::Link* link);

  /* Given a child link, build up the corresponding JointModel object. */
  JointModel* constructJointModel(const urdf::Link* child_link);

  /* Given a urdf link, build the corresponding LinkModel object. */
  LinkModel* constructLinkModel(const urdf::Link* urdf_link);

  /* Given a geometry spec from the URDF and a filename (for a mesh), construct the corresponding shape object. */
  shapes::ShapePtr constructShape(const urdf::Geometry* geom);
};
}  // namespace tobas
