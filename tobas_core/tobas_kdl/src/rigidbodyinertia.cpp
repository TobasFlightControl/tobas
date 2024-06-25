#include "../include/tobas_kdl/rigidbodyinertia.hpp"

#define E3 Matrix3d::Identity()

using namespace std;
using namespace Eigen;

namespace kdl
{
RigidBodyInertia::RigidBodyInertia(double m, const Vector& oc, const RotationalInertia& Ic) : m_(m), h_(m * oc)
{
  const auto& c_eig = oc.data;
  I_.data = Ic.data - m * (c_eig * c_eig.transpose() - c_eig.dot(c_eig) * E3);
}

RigidBodyInertia RigidBodyInertia::refPoint(const Vector& p) const
{
  const Vector hmr = h_ - m_ * p;
  const auto& r_eig = p.data;
  const auto& h_eig = h_.data;
  const auto& hmr_eig = hmr.data;
  const auto rcrosshcross = h_eig * r_eig.transpose() - r_eig.dot(h_eig) * E3;
  const auto hmrcrossrcross = r_eig * hmr_eig.transpose() - hmr_eig.dot(r_eig) * E3;
  const RotationalInertia Ib(I_.data + rcrosshcross + hmrcrossrcross);

  return RigidBodyInertia(m_, hmr, Ib, true);
}

RigidBodyInertia operator*(const Frame& T, const RigidBodyInertia& I)
{
  const Frame X = T.inverse();
  const Vector hmr = (I.h_ - I.m_ * X.p);
  const auto& R = T.M.data;
  const auto& r_eig = X.p.data;
  const auto& h_eig = I.h_.data;
  const auto& hmr_eig = hmr.data;
  const auto rcrosshcross = h_eig * r_eig.transpose() - r_eig.dot(h_eig) * E3;
  const auto hmrcrossrcross = r_eig * hmr_eig.transpose() - hmr_eig.dot(r_eig) * E3;
  const RotationalInertia Ib(R * (I.I_.data + rcrosshcross + hmrcrossrcross) * R.transpose());

  return RigidBodyInertia(I.m_, T.M * hmr, Ib, true);
}

RigidBodyInertia operator*(const Rotation& M, const RigidBodyInertia& I)
{
  const auto& R = M.data;
  const RotationalInertia Ib(R * I.I_.data * R.transpose());
  return RigidBodyInertia(I.m_, M * I.h_, Ib, true);
}
}  // namespace kdl
