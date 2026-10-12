// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base_view_model.hpp"
#include "./geometry_view_model.hpp"

namespace tobas
{
namespace gui
{
namespace ub
{
namespace view_model
{
class CollisionViewModel : public BaseViewModel<::urdf::Collision, CollisionViewModel>
{
public:
  explicit CollisionViewModel(const ::urdf::CollisionSharedPtr& _model);

  void sync() override;

  const ::urdf::Pose& origin() const;
  void origin(const ::urdf::Pose& _origin);

  QString name() const;
  void name(const QString& _name);

  const GeometryViewModelPtr& geometry();

private:
  GeometryViewModelPtr geometry_vm_;
};

using CollisionViewModelPtr = std::shared_ptr<CollisionViewModel>;
using V_CollisionViewModelPtr = std::vector<CollisionViewModelPtr>;
}  // namespace view_model
}  // namespace ub
}  // namespace gui
}  // namespace tobas
