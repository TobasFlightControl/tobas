#pragma once

#include <memory>

#include "./base_view_model.hpp"

namespace urdf_builder
{
namespace view_model
{
class JointLimitsViewModel : public BaseViewModel<urdf::JointLimits, JointLimitsViewModel>
{
public:
  using BaseViewModel<urdf::JointLimits, JointLimitsViewModel>::BaseViewModel;

  double lower() const
  {
    return model_->lower;
  }

  void lower(double lower)
  {
    model_->lower = lower;
  }

  double upper() const
  {
    return model_->upper;
  }

  void upper(double upper)
  {
    model_->upper = upper;
  }

  double effort() const
  {
    return model_->effort;
  }

  void effort(double effort)
  {
    model_->effort = effort;
  }

  double velocity() const
  {
    return model_->velocity;
  }

  void velocity(double velocity)
  {
    model_->velocity = velocity;
  }
};

using JointLimitsViewModelPtr = std::shared_ptr<JointLimitsViewModel>;
}  // namespace view_model
}  // namespace urdf_builder
