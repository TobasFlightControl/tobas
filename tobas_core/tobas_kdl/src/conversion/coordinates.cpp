#include "../../include/tobas_kdl/conversion/coordinates.hpp"

#include "../../include/tobas_kdl/rotational_inertia.hpp"

namespace kdl
{
void vectorNedToNwu(const Vector& src, Vector& des)
{
  des.x(src.x());
  des.y(-src.y());
  des.z(-src.z());
}

void vectorNwuToNed(const Vector& src, Vector& des)
{
  vectorNedToNwu(src, des);
}

void vectorNedToNwu(Vector& arg)
{
  vectorNedToNwu(arg, arg);
}

void vectorNwuToNed(Vector& arg)
{
  vectorNwuToNed(arg, arg);
}

void twistNedToNwu(const Twist& src, Twist& des)
{
  vectorNedToNwu(src.vel, des.vel);
  vectorNedToNwu(src.rot, des.rot);
}

void twistNwuToNed(const Twist& src, Twist& des)
{
  twistNedToNwu(src, des);
}

void twistNedToNwu(Twist& arg)
{
  twistNedToNwu(arg, arg);
}

void twistNwuToNed(Twist& arg)
{
  twistNwuToNed(arg, arg);
}

void rotInertiaNedToNwu(const RotationalInertia& src, RotationalInertia& des)
{
  des.data(0, 0) = src.data(0, 0);   // xx
  des.data(0, 1) = -src.data(0, 1);  // xy
  des.data(0, 2) = -src.data(0, 2);  // xz
  des.data(1, 0) = -src.data(1, 0);  // yx
  des.data(1, 1) = src.data(1, 1);   // yy
  des.data(1, 2) = src.data(1, 2);   // yz
  des.data(2, 0) = -src.data(2, 0);  // zx
  des.data(2, 1) = src.data(2, 1);   // zy
  des.data(2, 2) = src.data(2, 2);   // zz
}

void rotInertiaNwuToNed(const RotationalInertia& src, RotationalInertia& des)
{
  rotInertiaNedToNwu(src, des);
}

void rotInertiaNedToNwu(RotationalInertia& arg)
{
  rotInertiaNedToNwu(arg, arg);
}

void rotInertiaNwuToNed(RotationalInertia& arg)
{
  rotInertiaNwuToNed(arg, arg);
}

void eulerNedToNwu(const Euler& src, Euler& des)
{
  des.roll = src.roll;
  des.pitch = -src.pitch;
  des.yaw = -src.yaw;
}

void eulerNwuToNed(const Euler& src, Euler& des)
{
  eulerNedToNwu(src, des);
}

void eulerNedToNwu(Euler& arg)
{
  eulerNedToNwu(arg, arg);
}

void eulerNwuToNed(Euler& arg)
{
  eulerNwuToNed(arg, arg);
}

void rotationNedToNwu(const kdl::Rotation& src, kdl::Rotation& des)
{
  des.data(0, 0) = src.data(0, 0);   // xx
  des.data(0, 1) = -src.data(0, 1);  // xy
  des.data(0, 2) = -src.data(0, 2);  // xz
  des.data(1, 0) = -src.data(1, 0);  // yx
  des.data(1, 1) = src.data(1, 1);   // yy
  des.data(1, 2) = src.data(1, 2);   // yz
  des.data(2, 0) = -src.data(2, 0);  // zx
  des.data(2, 1) = src.data(2, 1);   // zy
  des.data(2, 2) = src.data(2, 2);   // zz
}

void rotationNwuToNed(const kdl::Rotation& src, kdl::Rotation& des)
{
  rotationNedToNwu(src, des);
}

void rotationNedToNwu(kdl::Rotation& arg)
{
  rotationNedToNwu(arg, arg);
}

void rotationNwuToNed(kdl::Rotation& arg)
{
  rotationNwuToNed(arg, arg);
}

void frameNedToNwu(const kdl::Frame& src, kdl::Frame& des)
{
  vectorNedToNwu(src.p, des.p);
  rotationNedToNwu(src.M, des.M);
}

void frameNwuToNed(const kdl::Frame& src, kdl::Frame& des)
{
  frameNedToNwu(src, des);
}

void frameNedToNwu(kdl::Frame& arg)
{
  frameNedToNwu(arg, arg);
}

void frameNwuToNed(kdl::Frame& arg)
{
  frameNwuToNed(arg, arg);
}
}  // namespace kdl
