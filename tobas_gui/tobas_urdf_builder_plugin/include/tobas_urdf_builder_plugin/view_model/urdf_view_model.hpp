// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tinyxml2.h>
#include <urdf/model.hpp>

#include "./joint_view_model.hpp"
#include "./link_view_model.hpp"

namespace tobas::gui::ub::view_model
{
class URDFViewModel
{
public:
  explicit URDFViewModel();

  /** Get URDF model. */
  const ::urdf::ModelSharedPtr& urdf() const;

  /** Get the complete list of links. */
  const std::map<std::string, ::urdf::LinkSharedPtr>& links() const;

  /** Get the complete list of joints. */
  const std::map<std::string, ::urdf::JointSharedPtr>& joints() const;

  /** Get the complete list of materials. */
  const std::map<std::string, ::urdf::MaterialSharedPtr>& materials() const;

  /** Get the name of the robot model. */
  const std::string& name() const;
  /** Set the name of the robot model. */
  void name(const std::string& _name);

  /** Get the root link (the parent of the tree describing the robot). */
  const ::urdf::LinkSharedPtr& rootLink() const;

  const LinkViewModelPtr& rootLinkViewModel() const;

  QStringList linkNames() const;
  QStringList jointNames() const;

  void newRobot();
  bool loadRobot(const QString& _file_path);
  bool saveRobot(const QString& _file_path);

  void addLink(const LinkViewModelPtr& _link_vm);
  void cloneLink(const LinkViewModelPtr& _link_vm);
  void removeLink(const LinkViewModelPtr& _link_vm);
  void updateLink(const LinkViewModelPtr& _old_link_vm, const LinkViewModelPtr& _new_link_vm);

private:
  ::urdf::ModelSharedPtr urdf_;
  LinkViewModelPtr root_link_;
  size_t clone_count_ = 0;

  /** Add a suffix to all links and joints under this node. */
  void addNameSuffixRec(const LinkViewModelPtr& _link_vm, const QString& _suffix);

  static void removeTextureTagsWithoutFilename(tinyxml2::XMLElement* _element);
};
}  // namespace tobas::gui::ub::view_model
