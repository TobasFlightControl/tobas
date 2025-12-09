#include "tobas_kdl/conversion/coordinates.hpp"

#include "tobas_kdl/rotational_inertia.hpp"

namespace kdl
{
void vectorFrdToFlu(const Vector& src, Vector& des)
{
  des.x(src.x());
  des.y(-src.y());
  des.z(-src.z());
}

void vectorFluToFrd(const Vector& src, Vector& des)
{
  vectorFrdToFlu(src, des);
}

void vectorFrdToFlu(Vector& arg)
{
  vectorFrdToFlu(arg, arg);
}

void vectorFluToFrd(Vector& arg)
{
  vectorFluToFrd(arg, arg);
}

void twistFrdToFlu(const Twist& src, Twist& des)
{
  vectorFrdToFlu(src.vel, des.vel);
  vectorFrdToFlu(src.rot, des.rot);
}

void twistFluToFrd(const Twist& src, Twist& des)
{
  twistFrdToFlu(src, des);
}

void twistFrdToFlu(Twist& arg)
{
  twistFrdToFlu(arg, arg);
}

void twistFluToFrd(Twist& arg)
{
  twistFluToFrd(arg, arg);
}

void rotInertiaFrdToFlu(const RotationalInertia& src, RotationalInertia& des)
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

void rotInertiaFluToFrd(const RotationalInertia& src, RotationalInertia& des)
{
  rotInertiaFrdToFlu(src, des);
}

void rotInertiaFrdToFlu(RotationalInertia& arg)
{
  rotInertiaFrdToFlu(arg, arg);
}

void rotInertiaFluToFrd(RotationalInertia& arg)
{
  rotInertiaFluToFrd(arg, arg);
}

void eulerFrdToFlu(const Euler& src, Euler& des)
{
  des.roll = src.roll;
  des.pitch = -src.pitch;
  des.yaw = -src.yaw;
}

void eulerFluToFrd(const Euler& src, Euler& des)
{
  eulerFrdToFlu(src, des);
}

void eulerFrdToFlu(Euler& arg)
{
  eulerFrdToFlu(arg, arg);
}

void eulerFluToFrd(Euler& arg)
{
  eulerFluToFrd(arg, arg);
}

void rotationFrdToFlu(const kdl::Rotation& src, kdl::Rotation& des)
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

void rotationFluToFrd(const kdl::Rotation& src, kdl::Rotation& des)
{
  rotationFrdToFlu(src, des);
}

void rotationFrdToFlu(kdl::Rotation& arg)
{
  rotationFrdToFlu(arg, arg);
}

void rotationFluToFrd(kdl::Rotation& arg)
{
  rotationFluToFrd(arg, arg);
}

void frameFrdToFlu(const kdl::Frame& src, kdl::Frame& des)
{
  vectorFrdToFlu(src.p, des.p);
  rotationFrdToFlu(src.M, des.M);
}

void frameFluToFrd(const kdl::Frame& src, kdl::Frame& des)
{
  frameFrdToFlu(src, des);
}

void frameFrdToFlu(kdl::Frame& arg)
{
  frameFrdToFlu(arg, arg);
}

void frameFluToFrd(kdl::Frame& arg)
{
  frameFluToFrd(arg, arg);
}
}  // namespace kdl
