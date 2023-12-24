#include <iostream>

#include <tobas_std_tools/math.hpp>

#include "../include/tobas_kdl/trajectory.hpp"

using namespace std;
using namespace KDL;

namespace tj
{
CycloidGenerator3d::CycloidGenerator3d()
{
}

void CycloidGenerator3d::generate(
  const KDL::Vector& p0,
  const KDL::Vector& pf,
  const double& T,
  const double& h,
  const double& k)
{
  assert(T > 0.);
  assert(h > 0.);
  assert(k >= 0.);

  p0_ = p0;
  pf_ = pf;
  T_ = T;
  h_ = h;
  k_ = k;
  TT_ = T * T;
  kk_ = k * k;
  p_diff_ = pf - p0;
}

void CycloidGenerator3d::get(const double& t, const Rotation& r, Vector& p, Vector& v, Vector& a)
{
  assert(t >= 0.);

  if (t <= T_)
  {
    getPos(t, r, p);
    getVel(t, r, v);
    getAcc(t, r, a);
  }
  else
  {
    p = r * pf_;
    v.setZero();
    a.setZero();
  }
}

void CycloidGenerator3d::get(const double& t, Vector& p, Vector& v, Vector& a)
{
  return get(t, r0_, p, v, a);
}

void CycloidGenerator3d::get(const double& t, Vector& p, Vector& v)
{
  return get(t, r0_, p, v, dummy_vector_);
}

void CycloidGenerator3d::get(const double& t, Vector& p)
{
  return get(t, r0_, p, dummy_vector_, dummy_vector_);
}

void CycloidGenerator3d::getPos(const double& t, const KDL::Rotation& r, Vector& p)
{
  const double theta = 2. * M_PI * t / T_;
  const double tmp = (theta - sin(theta)) / (2. * M_PI);

  p.x(p0_.x() + p_diff_.x() * tmp);
  p.y(p0_.y() + p_diff_.y() * tmp);
  p.z(pf_.z() + h_ / 2 * (1 - cos(theta)) - p_diff_.z() * exp(-k_ * t / T_));

  p = r * p;
}

void CycloidGenerator3d::getVel(const double& t, const KDL::Rotation& r, Vector& v)
{
  const double theta = 2. * M_PI * t / T_;
  const double tmp = (1. - cos(theta)) / T_;

  v.x(p_diff_.x() * tmp);
  v.y(p_diff_.y() * tmp);
  v.z(M_PI * h_ * sin(theta) / T_ + p_diff_.z() * k_ * exp(-k_ * t / T_) / T_);

  v = r * v;
}

void CycloidGenerator3d::getAcc(const double& t, const KDL::Rotation& r, Vector& a)
{
  const double theta = 2. * M_PI * t / T_;
  const double tmp = 2. * M_PI / TT_ * sin(theta);

  a.x(p_diff_.x() * tmp);
  a.y(p_diff_.y() * tmp);
  a.z(
    2 * tobas_std::sqr(M_PI) * h_ / TT_ * cos(theta) - p_diff_.z() * kk_ / TT_ * exp(-k_ * t / T_));

  a = r * a;
}
}  // namespace tj
