// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/view_model/joint_view_model.hpp"

namespace tobas
{
namespace gui
{
namespace ub
{
namespace view_model
{
JointViewModel::JointViewModel(const ::urdf::JointSharedPtr& _model)
  : BaseViewModel<::urdf::Joint, JointViewModel>(_model)
{
  if (model_->type == ::urdf::Joint::UNKNOWN) {
    model_->type = ::urdf::Joint::FIXED;
  }

  if (limitsEnabled()) {
    limits_.reset(new JointLimitsViewModel(model_->limits));
  }
}

void JointViewModel::sync()
{
  if (limitsEnabled()) {
    model_->limits = limits_->model();
  }
}

QString JointViewModel::name() const
{
  return QString::fromStdString(model_->name);
}

void JointViewModel::name(const QString& _name)
{
  model_->name = _name.toStdString();
}

QString JointViewModel::type() const
{
  switch (model_->type) {
    case ::urdf::Joint::UNKNOWN:
      return "Unknown";
    case ::urdf::Joint::REVOLUTE:
      return "Revolute";
    case ::urdf::Joint::CONTINUOUS:
      return "Continuous";
    case ::urdf::Joint::PRISMATIC:
      return "Prismatic";
    case ::urdf::Joint::FLOATING:
      return "Floating";
    case ::urdf::Joint::PLANAR:
      return "Planar";
    case ::urdf::Joint::FIXED:
      return "Fixed";
    default:
      throw;
  }
}

void JointViewModel::type(const QString& _type)
{
  if (_type == "Unknown") {
    model_->type = ::urdf::Joint::UNKNOWN;
  }
  else if (_type == "Revolute") {
    model_->type = ::urdf::Joint::REVOLUTE;
  }
  else if (_type == "Continuous") {
    model_->type = ::urdf::Joint::CONTINUOUS;
  }
  else if (_type == "Prismatic") {
    model_->type = ::urdf::Joint::PRISMATIC;
  }
  else if (_type == "Floating") {
    model_->type = ::urdf::Joint::FLOATING;
  }
  else if (_type == "Planar") {
    model_->type = ::urdf::Joint::PLANAR;
  }
  else if (_type == "Fixed") {
    model_->type = ::urdf::Joint::FIXED;
  }
  else {
    throw std::runtime_error("Invalid joint type: " + _type.toStdString());
  }

  if (limitsEnabled()) {
    limits_.reset(new JointLimitsViewModel(model_->limits));
  }
  else {
    limits_.reset();
  }
}

const ::urdf::Pose& JointViewModel::origin() const
{
  return model_->parent_to_joint_origin_transform;
}

void JointViewModel::origin(const ::urdf::Pose& _origin)
{
  model_->parent_to_joint_origin_transform = _origin;
}

QString JointViewModel::parentLinkName() const
{
  return QString::fromStdString(model_->parent_link_name);
}

void JointViewModel::parentLinkName(const QString& _name)
{
  model_->parent_link_name = _name.toStdString();
}

QString JointViewModel::childLinkName() const
{
  return QString::fromStdString(model_->child_link_name);
}

void JointViewModel::childLinkName(const QString& _name)
{
  model_->child_link_name = _name.toStdString();
}

const ::urdf::Vector3& JointViewModel::axis() const
{
  return model_->axis;
}

void JointViewModel::axis(const ::urdf::Vector3& _axis)
{
  model_->axis = _axis;
}

const JointLimitsViewModelPtr& JointViewModel::limits()
{
  return limits_;
}

bool JointViewModel::limitsEnabled() const
{
  return model_->type == ::urdf::Joint::REVOLUTE || model_->type == ::urdf::Joint::PRISMATIC;
}

bool JointViewModel::isFixed() const
{
  return model_->type == ::urdf::Joint::FIXED;
}
}  // namespace view_model
}  // namespace ub
}  // namespace gui
}  // namespace tobas
