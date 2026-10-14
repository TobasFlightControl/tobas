// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/view_model/collision_view_model.hpp"

#include "tobas_urdf_builder_plugin/utils/time.hpp"

namespace tobas::gui::ub::view_model
{
CollisionViewModel::CollisionViewModel(const ::urdf::CollisionSharedPtr& _model)
  : BaseViewModel<::urdf::Collision, CollisionViewModel>(_model)
  , geometry_vm_(std::make_shared<GeometryViewModel>(model_->geometry))
{
  if (model_->name.empty()) {
    model_->name = "collision_" + std::to_string(utils::timeNowMilliseconds());
  }
}

void CollisionViewModel::sync()
{
  geometry_vm_->sync();
  model_->geometry = geometry_vm_->model();
}

const ::urdf::Pose& CollisionViewModel::origin() const
{
  return model_->origin;
}

void CollisionViewModel::origin(const ::urdf::Pose& _origin)
{
  model_->origin = _origin;
}

QString CollisionViewModel::name() const
{
  return QString::fromStdString(model_->name);
}

void CollisionViewModel::name(const QString& _name)
{
  model_->name = _name.toStdString();
}

const GeometryViewModelPtr& CollisionViewModel::geometry()
{
  return geometry_vm_;
}
}  // namespace tobas::gui::ub::view_model
