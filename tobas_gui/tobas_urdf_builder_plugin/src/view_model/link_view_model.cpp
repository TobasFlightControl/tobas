// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/view_model/link_view_model.hpp"

namespace tobas::gui::ub::view_model
{
LinkViewModel::LinkViewModel(const ::urdf::LinkSharedPtr& _model)
  : BaseViewModel<::urdf::Link, LinkViewModel>(_model)
  , inertial_(make_shared<InertialViewModel>(model_->inertial))
  , joint_(make_shared<JointViewModel>(model_->parent_joint))
{
  for (const auto& visual : model_->visual_array) {
    visuals_.emplace_back(new VisualViewModel(visual));
  }

  for (const auto& collision : model_->collision_array) {
    collisions_.emplace_back(new CollisionViewModel(collision));
  }

  // Synchronize with the URDF in the constructor.
  // Otherwise, cloning a mesh causes a path error.
  sync();
}

void LinkViewModel::sync()
{
  inertial_->sync();
  joint_->sync();
  for (const auto& visual : visuals_) {
    visual->sync();
  }
  for (const auto& collision : collisions_) {
    collision->sync();
  }

  // inertial
  model_->inertial = inertial_->model();

  // collision, collision_array
  model_->collision_array.clear();
  std::transform(
    collisions_.begin(),
    collisions_.end(),
    back_inserter(model_->collision_array),
    [](const CollisionViewModelPtr& _cvm) { return _cvm->model(); });
  if (model_->collision_array.empty()) {
    model_->collision.reset();
  }
  else {
    model_->collision = model_->collision_array.front();
  }

  // visual, visual_array
  model_->visual_array.clear();
  std::transform(
    visuals_.begin(),
    visuals_.end(),
    back_inserter(model_->visual_array),
    [](const VisualViewModelPtr& _vvm) { return _vvm->model(); });
  if (model_->visual_array.empty()) {
    model_->visual.reset();
  }
  else {
    model_->visual = model_->visual_array.front();
  }

  // parent_joint
  model_->parent_joint = joint_->model();

  // Update `child_joints` and `child_links` whenever they change.
}

QString LinkViewModel::name() const
{
  return QString::fromStdString(model_->name);
}

void LinkViewModel::name(const QString& _name)
{
  model_->name = _name.toStdString();

  // Also change the link names in the upper and lower joints to keep consistency.
  joint_->childLinkName(_name);
  for (const auto& child_link : model_->child_links) {
    child_link->parent_joint->parent_link_name = _name.toStdString();
  }
}

const InertialViewModelPtr& LinkViewModel::inertial()
{
  return inertial_;
}

const V_VisualViewModelPtr& LinkViewModel::visuals()
{
  return visuals_;
}

const V_CollisionViewModelPtr& LinkViewModel::collisions()
{
  return collisions_;
}

const JointViewModelPtr& LinkViewModel::joint()
{
  return joint_;
}

V_LinkViewModelPtr LinkViewModel::children() const
{
  V_LinkViewModelPtr result;
  for (const auto& child : model_->child_links) {
    result.emplace_back(new LinkViewModel(child));
  }
  return result;
}

void LinkViewModel::add(const VisualViewModelPtr& _visual)
{
  visuals_.push_back(_visual);

  sync();
}

void LinkViewModel::remove(const VisualViewModelPtr& _visual)
{
  visuals_.erase(std::remove(visuals_.begin(), visuals_.end(), _visual), visuals_.end());

  sync();
}

void LinkViewModel::add(const CollisionViewModelPtr& _collision)
{
  collisions_.push_back(_collision);

  sync();
}

void LinkViewModel::remove(const CollisionViewModelPtr& _collision)
{
  collisions_.erase(std::remove(collisions_.begin(), collisions_.end(), _collision), collisions_.end());

  sync();
}
}  // namespace tobas::gui::ub::view_model
