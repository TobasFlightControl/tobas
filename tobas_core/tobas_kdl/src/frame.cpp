#include "../include/tobas_kdl/frame.hpp"

using namespace std;

namespace KDL
{
Frame Frame::DH_Craig1989(double a, double alpha, double d, double theta)
{
  const double ct = cos(theta);
  const double st = sin(theta);
  const double sa = sin(alpha);
  const double ca = cos(alpha);
  return Frame(
    Rotation(ct, -st, 0, st * ca, ct * ca, -sa, st * sa, ct * sa, ca), Vector(a, -sa * d, ca * d));
}

Frame Frame::DH(double a, double alpha, double d, double theta)
{
  const double ct = cos(theta);
  const double st = sin(theta);
  const double sa = sin(alpha);
  const double ca = cos(alpha);
  return Frame(
    Rotation(ct, -st * ca, st * sa, st, ct * ca, -ct * sa, 0, sa, ca), Vector(a * ct, a * st, d));
}

void Frame::Make4x4(double* d)
{
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
      d[i * 4 + j] = M(i, j);
    d[i * 4 + 3] = p(i);
  }
  for (int j = 0; j < 3; ++j)
    d[12 + j] = 0.;
  d[15] = 1;
}

void Frame::integrate(const Twist& t_this, double samplefrequency)
{
  const double n = t_this.rot.norm() / samplefrequency;
  if (n < kDefaultEpsilon)
    p += M * (t_this.vel / samplefrequency);
  else
    (*this) = (*this) * Frame(Rotation::Rot(t_this.rot, n), t_this.vel / samplefrequency);
}

Twist Frame::toTwist() const
{
  return Twist(p, M.getRot());
}
}  // namespace KDL
