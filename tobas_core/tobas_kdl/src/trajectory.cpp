#include "../include/tobas_kdl/trajectory.hpp"

#include <iostream>

#include <tobas_math/core.hpp>

using namespace std;

namespace kdl
{
CycloidGenerator3d::CycloidGenerator3d()
{
}

bool CycloidGenerator3d::generate(
  const kdl::Vector& p0,
  const kdl::Vector& pf,
  const double& T,
  const double& h,
  const double& k)
{
  if (T <= 0) {
    cerr << "The period T must be positive." << endl;
    return false;
  }
  if (h <= 0) {
    cerr << " The height h must be positive." << endl;
    return false;
  }
  if (k < 0) {
    cerr << "The order k must be non-negative." << endl;
    return false;
  }

  p0_ = p0;
  pf_ = pf;
  T_ = T;
  h_ = h;
  k_ = k;
  TT_ = math::sqr(T);
  kk_ = math::sqr(k);
  p_diff_ = pf - p0;

  return true;
}

bool CycloidGenerator3d::get(const double& t, const Rotation& r, Vector& p, Vector& v, Vector& a) const
{
  if (t < 0) {
    cerr << "The time t must be non-negative." << endl;
    return false;
  }

  if (t <= T_) {
    getPos(t, r, p);
    getVel(t, r, v);
    getAcc(t, r, a);
  }
  else {
    p = r * pf_;
    v.setZero();
    a.setZero();
  }

  return true;
}

bool CycloidGenerator3d::get(const double& t, Vector& p, Vector& v, Vector& a) const
{
  return get(t, r0_, p, v, a);
}

bool CycloidGenerator3d::get(const double& t, Vector& p, Vector& v) const
{
  Vector dummy_vector;
  return get(t, r0_, p, v, dummy_vector);
}

bool CycloidGenerator3d::get(const double& t, Vector& p) const
{
  Vector dummy_vector;
  return get(t, r0_, p, dummy_vector, dummy_vector);
}

void CycloidGenerator3d::getPos(const double& t, const kdl::Rotation& r, Vector& p) const
{
  const auto theta = computeTheta(t);
  const auto tmp = (theta - sin(theta)) / (2 * M_PI);

  p.x(p0_.x() + p_diff_.x() * tmp);
  p.y(p0_.y() + p_diff_.y() * tmp);
  p.z(pf_.z() + h_ / 2 * (1 - cos(theta)) - p_diff_.z() * exp(-k_ * t / T_));

  p = r * p;
}

void CycloidGenerator3d::getVel(const double& t, const kdl::Rotation& r, Vector& v) const
{
  const auto theta = computeTheta(t);
  const auto tmp = (1 - cos(theta)) / T_;

  v.x(p_diff_.x() * tmp);
  v.y(p_diff_.y() * tmp);
  v.z(M_PI * h_ * sin(theta) / T_ + p_diff_.z() * k_ * exp(-k_ * t / T_) / T_);

  v = r * v;
}

void CycloidGenerator3d::getAcc(const double& t, const kdl::Rotation& r, Vector& a) const
{
  const auto theta = computeTheta(t);
  const auto tmp = 2 * M_PI / TT_ * sin(theta);

  a.x(p_diff_.x() * tmp);
  a.y(p_diff_.y() * tmp);
  a.z(2 * math::sqr(M_PI) * h_ / TT_ * cos(theta) - p_diff_.z() * kk_ / TT_ * exp(-k_ * t / T_));

  a = r * a;
}

double CycloidGenerator3d::computeTheta(const double& t) const
{
  assert(t >= 0);
  return 2 * M_PI * t / T_;
}
}  // namespace kdl
