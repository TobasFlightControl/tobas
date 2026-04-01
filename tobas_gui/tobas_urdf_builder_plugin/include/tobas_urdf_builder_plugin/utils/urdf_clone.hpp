// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <urdf_model/joint.h>
#include <urdf_model/link.h>

namespace tobas
{
namespace gui
{
namespace ub
{
namespace utils
{
/**
 * @brief デフォルトのクローン関数．
 * インスタンス変数にポインタが含まれる場合，make_sharedではその実態はコピーされないため特殊化 or オーバーロードが必要．
 */
template <typename T>
std::shared_ptr<T> clone(const std::shared_ptr<T>& ptr)
{
  if (!ptr) {
    return nullptr;
  }

  return std::make_shared<T>(*ptr);
}

::urdf::GeometrySharedPtr clone(const ::urdf::GeometrySharedPtr& geometry);
::urdf::VisualSharedPtr clone(const ::urdf::VisualSharedPtr& visual);
::urdf::CollisionSharedPtr clone(const ::urdf::CollisionSharedPtr& collision);
::urdf::JointCalibrationSharedPtr clone(const ::urdf::JointCalibrationSharedPtr& calibration);
::urdf::JointSharedPtr clone(const ::urdf::JointSharedPtr& joint);
::urdf::LinkSharedPtr clone(const ::urdf::LinkSharedPtr& link);
}  // namespace utils
}  // namespace ub
}  // namespace gui
}  // namespace tobas
