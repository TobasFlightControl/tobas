#pragma once

#include "./base_view_model.hpp"

namespace urdf_builder
{
namespace view_model
{
class JointLimitsViewModel : public BaseViewModel<urdf::JointLimits, JointLimitsViewModel>
{
public:
  using BaseViewModel<urdf::JointLimits, JointLimitsViewModel>::BaseViewModel;

  void sync() override;

  double lower() const;
  void lower(double lower);

  double upper() const;
  void upper(double upper);

  double effort() const;
  void effort(double effort);

  double velocity() const;
  void velocity(double velocity);
};

using JointLimitsViewModelPtr = std::shared_ptr<JointLimitsViewModel>;
}  // namespace view_model
}  // namespace urdf_builder
