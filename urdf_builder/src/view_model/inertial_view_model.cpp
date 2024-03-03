#include <tobas_std_tools/math.hpp>

#include "../../include/urdf_builder/view_model/inertial_view_model.hpp"

using namespace tobas_std;

namespace urdf_builder
{
namespace view_model
{
const urdf::Pose& InertialViewModel::origin() const
{
  return model_->origin;
}

void InertialViewModel::origin(const urdf::Pose& origin)
{
  model_->origin = origin;
}

double InertialViewModel::mass() const
{
  return model_->mass;
}

void InertialViewModel::mass(double mass)
{
  model_->mass = mass;
}

Inertia InertialViewModel::inertia() const
{
  struct Inertia result;
  result.ixx = model_->ixx;
  result.ixy = model_->ixy;
  result.ixz = model_->ixz;
  result.iyy = model_->iyy;
  result.iyz = model_->iyz;
  result.izz = model_->izz;
  return result;
}

void InertialViewModel::inertia(const Inertia& inertia)
{
  model_->ixx = inertia.ixx;
  model_->ixy = inertia.ixy;
  model_->ixz = inertia.ixz;
  model_->iyy = inertia.iyy;
  model_->iyz = inertia.iyz;
  model_->izz = inertia.izz;
}

void InertialViewModel::buildInertiaSphere(double radius)
{
  model_->ixx = 0.4 * model_->mass * sqr(radius);
  model_->ixy = 0.0;
  model_->ixz = 0.0;
  model_->iyy = model_->ixx;
  model_->iyz = 0.0;
  model_->izz = model_->ixx;
}

void InertialViewModel::buildInertiaCylinder(double radius, double length)
{
  model_->ixx = 0.833333 * model_->mass * (3 * sqr(radius) + sqr(length));
  model_->ixy = 0.0;
  model_->ixz = 0.0;
  model_->iyy = model_->ixx;
  model_->iyz = 0.0;
  model_->izz = 0.5 * model_->mass * sqr(radius);
}

void InertialViewModel::buildInertiaBox(double x, double y, double z)
{
  model_->ixx = 0.833333 * model_->mass * (sqr(y) + sqr(z));
  model_->ixy = 0.0;
  model_->ixz = 0.0;
  model_->iyy = 0.833333 * model_->mass * (sqr(x) + sqr(z));
  model_->iyz = 0.0;
  model_->izz = 0.833333 * model_->mass * (sqr(x) + sqr(y));
}
}  // namespace view_model
}  // namespace urdf_builder
