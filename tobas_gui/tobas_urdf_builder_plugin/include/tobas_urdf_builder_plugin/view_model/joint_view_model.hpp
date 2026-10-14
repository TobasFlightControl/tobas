// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <QString>

#include "./base_view_model.hpp"
#include "./joint_limits_view_model.hpp"

namespace tobas
{
namespace gui
{
namespace ub
{
namespace view_model
{
class JointViewModel : public BaseViewModel<::urdf::Joint, JointViewModel>
{
public:
  explicit JointViewModel(const ::urdf::JointSharedPtr& _model);

  void sync() override;

  QString name() const;
  void name(const QString& _name);

  QString type() const;
  void type(const QString& _type);

  const ::urdf::Pose& origin() const;
  void origin(const ::urdf::Pose& _origin);

  QString parentLinkName() const;
  void parentLinkName(const QString& _name);

  QString childLinkName() const;
  void childLinkName(const QString& _name);

  const ::urdf::Vector3& axis() const;
  void axis(const ::urdf::Vector3& _axis);

  const JointLimitsViewModelPtr& limits();

  bool limitsEnabled() const;
  bool isFixed() const;

private:
  JointLimitsViewModelPtr limits_;
};

using JointViewModelPtr = std::shared_ptr<JointViewModel>;
using V_JointViewModelPtr = std::vector<JointViewModelPtr>;
}  // namespace view_model
}  // namespace ub
}  // namespace gui
}  // namespace tobas
