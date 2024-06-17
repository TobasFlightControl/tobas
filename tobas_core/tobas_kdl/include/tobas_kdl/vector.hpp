#pragma once

#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "./utilities/constants.hpp"

namespace tobas_kdl
{
class Vector;
using VectorMap = std::map<std::string, Vector>;

/**
 * \brief A concrete implementation of a 3 dimensional vector class.
 */
class Vector
{
public:
  Eigen::Vector3d data;

  /* Does not initialise the Vector to zero. use Vector::Zero() or setToZero for that. */
  inline explicit Vector();
  inline explicit Vector(double x, double y, double z);
  inline explicit Vector(const Eigen::Vector3d& data);

  inline static Vector Zero();
  inline static Vector Constant(const double& value);
  inline static Vector UnitX();
  inline static Vector UnitY();
  inline static Vector UnitZ();

  /* Access to elements, range checked when NDEBUG is not set, from 0..2 */
  inline double operator()(size_t index) const;
  /* Access to elements, range checked when NDEBUG is not set, from 0..2 */
  inline double& operator()(size_t index);

  inline const double& x() const;
  inline const double& y() const;
  inline const double& z() const;
  inline double& x();
  inline double& y();
  inline double& z();
  inline void x(double x);
  inline void y(double y);
  inline void z(double z);

  inline void fill(double value);
  inline double dot(const Vector& rhs) const;
  inline Vector hadamard(const Vector& rhs) const;
  inline bool contains(double value) const;

  /* Clamp each value. */
  inline Vector clamp(const double& lb, const double& ub) const;
  inline Vector clamp(const Vector& lb, const Vector& ub) const;

  inline Vector inverse() const;

  /* Adds a vector from the Vector object itself. */
  inline Vector& operator+=(const Vector& arg);
  /* Subtracts a vector from the Vector object itself. */
  inline Vector& operator-=(const Vector& arg);

  inline friend Vector operator-(const Vector& arg);
  inline friend Vector operator+(const Vector& lhs, double rhs);
  inline friend Vector operator+(double lhs, const Vector& rhs);
  inline friend Vector operator-(const Vector& lhs, double rhs);
  inline friend Vector operator-(double lhs, const Vector& rhs);
  inline friend Vector operator*(const Vector& lhs, double rhs);
  inline friend Vector operator*(double lhs, const Vector& rhs);
  inline friend Vector operator/(const Vector& lhs, double rhs);
  inline friend Vector operator/(double lhs, const Vector& rhs);
  inline friend Vector operator+(const Vector& lhs, const Vector& rhs);
  inline friend Vector operator-(const Vector& lhs, const Vector& rhs);
  inline friend Vector operator*(const Vector& lhs, const Vector& rhs);

  inline void setZero();
  /* To have a uniform operator to put an element to zero, for scalar values and for objects. */
  inline friend void setToZero(Vector& v);

  /* The norm of the vector */
  double norm(double eps = kDefaultEpsilon) const;

  /** Normalizes this vector and returns it norm
   * makes v a unitvector and returns the norm of v.
   * if v is smaller than eps, Vector(1,0,0) is returned with norm 0.
   * if this is not good, check the return value of this method.
   */
  double normalize(double eps = kDefaultEpsilon);
  Vector normalized() const;

  bool isFinite() const;

  friend std::ostream& operator<<(std::ostream& os, const Vector& arg);
};

inline Vector::Vector()
{
}

inline Vector::Vector(double x, double y, double z) : data(x, y, z)
{
}

inline Vector::Vector(const Eigen::Vector3d& data) : data(data)
{
}

inline Vector Vector::Zero()
{
  return Vector(Eigen::Vector3d::Zero());
}

inline Vector Vector::Constant(const double& value)
{
  return Vector(Eigen::Vector3d::Constant(value));
}

inline Vector Vector::UnitX()
{
  return Vector(Eigen::Vector3d::UnitX());
}

inline Vector Vector::UnitY()
{
  return Vector(Eigen::Vector3d::UnitY());
}

inline Vector Vector::UnitZ()
{
  return Vector(Eigen::Vector3d::UnitZ());
}

inline double Vector::operator()(size_t index) const
{
  return data(index);
}

inline double& Vector::operator()(size_t index)
{
  return data(index);
}

inline const double& Vector::x() const
{
  return data.x();
}

inline const double& Vector::y() const
{
  return data.y();
}

inline const double& Vector::z() const
{
  return data.z();
}

inline double& Vector::x()
{
  return data.x();
}

inline double& Vector::y()
{
  return data.y();
}

inline double& Vector::z()
{
  return data.z();
}

inline void Vector::x(double x)
{
  data.x() = x;
}

inline void Vector::y(double y)
{
  data.y() = y;
}

inline void Vector::z(double z)
{
  data.z() = z;
}

inline void Vector::fill(double value)
{
  data.fill(value);
}

inline double Vector::dot(const Vector& rhs) const
{
  return data.dot(rhs.data);
}

inline Vector Vector::hadamard(const Vector& rhs) const
{
  return Vector(data.cwiseProduct(rhs.data));
}

inline bool Vector::contains(double value) const
{
  return x() == value || y() == value || z() == value;
}

inline Vector Vector::clamp(const double& lb, const double& ub) const
{
  return Vector(data.cwiseMax(lb).cwiseMin(ub));
}

inline Vector Vector::clamp(const Vector& lb, const Vector& ub) const
{
  return Vector(data.cwiseMax(lb.data).cwiseMin(ub.data));
}

inline Vector Vector::inverse() const
{
  assert(x() != 0 && y() != 0 && z() != 0);
  return Vector(data.cwiseInverse());
}

inline Vector& Vector::operator+=(const Vector& arg)
{
  data += arg.data;
  return *this;
}

inline Vector& Vector::operator-=(const Vector& arg)
{
  data -= arg.data;
  return *this;
}

inline Vector operator-(const Vector& arg)
{
  return Vector(-arg.data);
}

inline Vector operator+(const Vector& lhs, double rhs)
{
  return Vector(lhs.data.array() + rhs);
}

inline Vector operator+(double lhs, const Vector& rhs)
{
  return Vector(lhs + rhs.data.array());
}

inline Vector operator-(const Vector& lhs, double rhs)
{
  return Vector(lhs.data.array() - rhs);
}

inline Vector operator-(double lhs, const Vector& rhs)
{
  return Vector(lhs - rhs.data.array());
}

inline Vector operator*(const Vector& lhs, double rhs)
{
  return Vector(lhs.data * rhs);
}

inline Vector operator*(double lhs, const Vector& rhs)
{
  return Vector(lhs * rhs.data);
}

inline Vector operator/(const Vector& lhs, double rhs)
{
  assert(rhs != 0);
  return Vector(lhs.data / rhs);
}

inline Vector operator/(double lhs, const Vector& rhs)
{
  return lhs * rhs.inverse();
}

inline Vector operator+(const Vector& lhs, const Vector& rhs)
{
  return Vector(lhs.data + rhs.data);
}

inline Vector operator-(const Vector& lhs, const Vector& rhs)
{
  return Vector(lhs.data - rhs.data);
}

inline Vector operator*(const Vector& lhs, const Vector& rhs)
{
  return Vector(lhs.data.cross(rhs.data));
}

inline void Vector::setZero()
{
  data.setZero();
}

inline void setToZero(Vector& v)
{
  v.data.setZero();
}
}  // namespace tobas_kdl
