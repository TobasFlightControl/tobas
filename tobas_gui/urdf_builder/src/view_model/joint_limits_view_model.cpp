#include "../../include/urdf_builder/view_model/joint_limits_view_model.hpp"

namespace urdf_builder
{
namespace view_model
{
void JointLimitsViewModel::sync()
{
}

double JointLimitsViewModel::lower() const
{
  return model_->lower;
}

void JointLimitsViewModel::lower(double lower)
{
  model_->lower = lower;
}

double JointLimitsViewModel::upper() const
{
  return model_->upper;
}

void JointLimitsViewModel::upper(double upper)
{
  model_->upper = upper;
}

double JointLimitsViewModel::effort() const
{
  return model_->effort;
}

void JointLimitsViewModel::effort(double effort)
{
  model_->effort = effort;
}

double JointLimitsViewModel::velocity() const
{
  return model_->velocity;
}

void JointLimitsViewModel::velocity(double velocity)
{
  model_->velocity = velocity;
}
}  // namespace view_model
}  // namespace urdf_builder
