// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/view_model/urdf_view_model.hpp"

#include <queue>

#include <QDebug>

#include <tobas_std_tools/vector.hpp>
#include <tobas_urdf/exporter.hpp>

namespace tobas::gui::ub::view_model
{
URDFViewModel::URDFViewModel() : urdf_(new ::urdf::Model())
{
}

const ::urdf::ModelSharedPtr& URDFViewModel::urdf() const
{
  return urdf_;
};

const std::map<std::string, ::urdf::LinkSharedPtr>& URDFViewModel::links() const
{
  return urdf_->links_;
}

const std::map<std::string, ::urdf::JointSharedPtr>& URDFViewModel::joints() const
{
  return urdf_->joints_;
}

const std::map<std::string, ::urdf::MaterialSharedPtr>& URDFViewModel::materials() const
{
  return urdf_->materials_;
}

const std::string& URDFViewModel::name() const
{
  return urdf_->name_;
}

void URDFViewModel::name(const std::string& _name)
{
  urdf_->name_ = _name;
}

const ::urdf::LinkSharedPtr& URDFViewModel::rootLink() const
{
  return urdf_->root_link_;
}

const LinkViewModelPtr& URDFViewModel::rootLinkViewModel() const
{
  return root_link_;
}

QStringList URDFViewModel::linkNames() const
{
  QStringList result;
  std::transform(
    urdf_->links_.begin(),
    urdf_->links_.end(),
    std::back_inserter(result),
    [](const std::pair<std::string, ::urdf::LinkSharedPtr>& _pair) { return QString::fromStdString(_pair.first); });
  return result;
}

QStringList URDFViewModel::jointNames() const
{
  QStringList result;
  std::transform(
    urdf_->joints_.begin(),
    urdf_->joints_.end(),
    std::back_inserter(result),
    [](const std::pair<std::string, ::urdf::JointSharedPtr>& _pair) { return QString::fromStdString(_pair.first); });
  return result;
}

void URDFViewModel::newRobot()
{
  urdf_.reset(new ::urdf::Model());
  root_link_.reset();
  clone_count_ = 0;
}

bool URDFViewModel::loadRobot(const QString& _file_path)
{
  newRobot();

  if (!urdf_->initFile(_file_path.toStdString())) {
    qWarning() << "Failed to parse URDF.";
    urdf_.reset();
    return false;
  }

  root_link_.reset(new LinkViewModel(urdf_->root_link_));
  return true;
}

bool URDFViewModel::saveRobot(const QString& _file_path)
{
  // Remove root-link inertia.
  urdf_->root_link_->inertial.reset();

  // Write the URDF.
  const auto doc = urdf::exportUrdf(*urdf_);

  // Delete unused textures.
  removeTextureTagsWithoutFilename(doc->RootElement());

  // Save XML.
  if (doc->SaveFile(_file_path.toStdString().c_str()) != tinyxml2::XML_SUCCESS) {
    qWarning() << "Failed to save URDF:" << doc->ErrorStr();
    return false;
  }

  return true;
}

void URDFViewModel::addLink(const LinkViewModelPtr& _link_vm)
{
  const auto& joint_vm = _link_vm->joint();

  const auto link_name = _link_vm->name().toStdString();
  const auto joint_name = joint_vm->name().toStdString();
  const auto parent_link_name = joint_vm->parentLinkName().toStdString();

  if (link_name.empty()) {
    throw std::runtime_error("Link name cannot be empty.");
  }

  if (urdf_->links_.find(link_name) != urdf_->links_.end()) {
    throw std::runtime_error("Link '" + link_name + "' already exists.");
  }

  if (urdf_->joints_.find(joint_name) != urdf_->joints_.end()) {
    throw std::runtime_error("Joint '" + joint_name + "' already exists.");
  }

  if (parent_link_name.empty()) {
    if (urdf_->root_link_) {
      throw std::runtime_error(
        "The root link already exists, but the parent link of '" + link_name + "' is not specified.");
    }

    urdf_->root_link_ = _link_vm->model();
    urdf_->joints_.erase(urdf_->root_link_->parent_joint->name);
    urdf_->root_link_->parent_joint.reset();
    urdf_->root_link_->inertial.reset();  // The root link cannot have inertia.
    root_link_.reset(new LinkViewModel(urdf_->root_link_));
  }
  else {
    if (joint_name.empty()) {
      throw std::runtime_error("Joint name cannot be empty.");
    }

    if (urdf_->links_.find(parent_link_name) == urdf_->links_.end()) {
      throw std::runtime_error("Parent link '" + parent_link_name + "' does not exist.");
    }

    const auto& parent_link = urdf_->links_.at(parent_link_name);
    auto& child_links = parent_link->child_links;
    auto& child_joints = parent_link->child_joints;
    if (!st::contains(child_links, _link_vm->model())) {
      child_links.push_back(_link_vm->model());
    }
    if (!st::contains(child_joints, joint_vm->model())) {
      child_joints.push_back(joint_vm->model());
    }
  }

  urdf_->links_[link_name] = _link_vm->model();
  urdf_->joints_[joint_name] = joint_vm->model();

  for (const auto& visual : _link_vm->visuals()) {
    const auto& material_vm = visual->material();
    urdf_->materials_[material_vm->name().toStdString()] = material_vm->model();
  }

  for (const auto& child_link_vm : _link_vm->children()) {
    addLink(child_link_vm);
  }
}

void URDFViewModel::cloneLink(const LinkViewModelPtr& _link_vm)
{
  const auto clone = _link_vm->clone();
  const auto suffix = "_" + QString::number(++clone_count_);
  addNameSuffixRec(clone, suffix);
  addLink(clone);
}

void URDFViewModel::removeLink(const LinkViewModelPtr& _link_vm)
{
  const auto& link = _link_vm->model();
  assert(link != urdf_->root_link_);  // Deleting the root link causes bugs.

  const auto& parent_link = urdf_->links_[link->parent_joint->parent_link_name];
  auto& child_links = parent_link->child_links;
  auto& child_joints = parent_link->child_joints;
  child_links.erase(remove(child_links.begin(), child_links.end(), link), child_links.end());
  child_joints.erase(remove(child_joints.begin(), child_joints.end(), link->parent_joint), child_joints.end());

  std::queue<::urdf::LinkSharedPtr> que;
  que.push(link);

  while (!que.empty()) {
    auto top = que.front();
    que.pop();

    urdf_->links_.erase(top->name);
    if (top->parent_joint) {
      urdf_->joints_.erase(top->parent_joint->name);
    }

    for (const auto& visual : top->visual_array) {
      urdf_->materials_.erase(visual->material_name);
    }

    for (const auto& child : top->child_links) {
      que.push(child);
    }
  }
}

void URDFViewModel::updateLink(const LinkViewModelPtr& _old_link_vm, const LinkViewModelPtr& _new_link_vm)
{
  // Remove old.
  const auto& old_joint = _old_link_vm->joint();

  urdf_->links_.erase(_old_link_vm->name().toStdString());
  urdf_->joints_.erase(old_joint->name().toStdString());
  for (const auto& visual : _old_link_vm->visuals()) {
    urdf_->materials_.erase(visual->material()->name().toStdString());
  }

  const auto& old_parent_link_it = urdf_->links_.find(old_joint->parentLinkName().toStdString());
  if (old_parent_link_it != urdf_->links_.end()) {
    const auto& old_parent_link = old_parent_link_it->second;

    auto& child_links = old_parent_link->child_links;
    const auto& it1 = remove_if(
      child_links.begin(),
      child_links.end(),
      [&](const ::urdf::LinkSharedPtr& _link) { return _link->name == _old_link_vm->name().toStdString(); });
    child_links.erase(it1, child_links.end());

    auto& child_joints = old_parent_link->child_joints;
    const auto& it2 = remove_if(
      child_joints.begin(),
      child_joints.end(),
      [&](const ::urdf::JointSharedPtr& _joint) { return _joint->name == old_joint->name().toStdString(); });
    child_joints.erase(it2, child_joints.end());
  }

  // Add new.
  const auto& new_joint = _new_link_vm->joint();
  const auto& new_parent_link_it = urdf_->links_.find(new_joint->parentLinkName().toStdString());
  if (new_parent_link_it != urdf_->links_.end()) {
    const auto& new_parent_link = new_parent_link_it->second;
    new_parent_link->child_links.push_back(_new_link_vm->model());
    new_parent_link->child_joints.push_back(_new_link_vm->joint()->model());
  }

  urdf_->joints_[new_joint->name().toStdString()] = new_joint->model();
  urdf_->links_[_new_link_vm->name().toStdString()] = _new_link_vm->model();
  for (const auto& visual : _new_link_vm->visuals()) {
    urdf_->materials_[visual->material()->name().toStdString()] = visual->material()->model();
  }

  if (_new_link_vm->model() == urdf_->root_link_) {
    urdf_->root_link_ = _new_link_vm->model();
    root_link_.reset(new LinkViewModel(urdf_->root_link_));
  }
}

void URDFViewModel::addNameSuffixRec(const LinkViewModelPtr& _link_vm, const QString& _suffix)
{
  const auto& joint_vm = _link_vm->joint();

  const auto new_link_name = _link_vm->name() + _suffix;
  const auto new_joint_name = joint_vm->name() + _suffix;

  _link_vm->name(new_link_name);
  joint_vm->name(new_joint_name);
  joint_vm->childLinkName(new_link_name);

  for (const auto& child_link_vm : _link_vm->children()) {
    child_link_vm->joint()->parentLinkName(new_link_name);
    addNameSuffixRec(child_link_vm, _suffix);
  }
}

void URDFViewModel::removeTextureTagsWithoutFilename(tinyxml2::XMLElement* _element)
{
  if (!_element) {
    return;
  }

  for (auto child = _element->FirstChildElement(); child; child = child->NextSiblingElement()) {
    // FIXME: Changing the tree structure during iteration may be unsafe. List target elements and delete them later.
    if (std::string(child->Name()) == "texture" && !child->Attribute("filename")) {
      _element->DeleteChild(child);
    }

    // Check child elements recursively.
    removeTextureTagsWithoutFilename(child);
  }
}
}  // namespace tobas::gui::ub::view_model
