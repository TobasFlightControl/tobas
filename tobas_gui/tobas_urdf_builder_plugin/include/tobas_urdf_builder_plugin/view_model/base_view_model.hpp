// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include "../utils/urdf_clone.hpp"

namespace tobas
{
namespace gui
{
namespace ub
{
namespace view_model
{
/**
 * @brief Wrapper for a URDF element class.
 *
 * @tparam M URDF element class.
 * @tparam Derived Derived class.
 */
template <typename M, typename Derived>
class BaseViewModel
{
public:
  using ModelPtr = std::shared_ptr<M>;
  using DerivedPtr = std::shared_ptr<Derived>;

  explicit BaseViewModel(const ModelPtr& model) : model_(model)
  {
    if (!model_) {
      model_.reset(new M());
    }
  }

  const ModelPtr& model()
  {
    return model_;
  }

  DerivedPtr clone() const
  {
    return DerivedPtr(new Derived(utils::clone(model_)));
  }

  /* Apply the View Model contents to the URDF model. */
  virtual void sync() = 0;

protected:
  ModelPtr model_;
};
}  // namespace view_model
}  // namespace ub
}  // namespace gui
}  // namespace tobas
