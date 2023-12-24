#pragma once

#include <memory>
#include <urdf_model/link.h>

#include "../utils/urdf_clone.hpp"

namespace urdf_builder
{
namespace view_model
{
template <typename M, typename Derived>
class BaseViewModel
{
public:
  using ModelPtr = std::shared_ptr<M>;
  using DerivedPtr = std::shared_ptr<Derived>;

  explicit BaseViewModel(const ModelPtr& model) : model_(model)
  {
    if (!model_)
      model_.reset(new M());
  }

  const ModelPtr& model() const
  {
    return model_;
  }

  DerivedPtr clone() const
  {
    return DerivedPtr(new Derived(utils::clone(model_)));
  }

  virtual void sync()
  {
  }

protected:
  ModelPtr model_;
};
}  // namespace view_model
}  // namespace urdf_builder
