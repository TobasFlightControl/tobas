#pragma once

#include <map>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <tobas_std_tools/float.hpp>
#include <tobas_eigen_tools/core.hpp>

namespace kdl
{
class Vector;
using VectorMap = std::map<std::string, Vector>;

/**
 * @brief A concrete implementation of a 3 dimensional vector class.
 */
class Vector
{
public:
  Eigen::Vector3d data;

  inline Vector();
  inline Vector(double x, double y, double z);
  inline explicit Vector(const Eigen::Vector3d& _data);

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

  /* 最小の要素を取得する． */
  inline double min() const;

  /* 最大の要素を取得する． */
  inline double max() const;

  /* 2つのベクトルの内積を計算する． */
  inline double dot(const Vector& rhs) const;

  /* 2つのベクトルの要素積を計算する． */
  inline Vector hadamard(const Vector& rhs) const;

  /* 2つのベクトル間の偏角 [rad] を計算する． */
  inline double argument(const Vector& rhs) const;

  /* 2つのベクトルが直行するかどうかを判定する． */
  inline bool isOrthogonal(const Vector& rhs) const;

  /* Clamp each value. */
  inline Vector clamp(const double& lb, const double& ub) const;
  inline Vector clamp(const Vector& lb, const Vector& ub) const;

  inline void setZero();

  inline double norm() const;
  inline void normalize();
  inline Vector normalized() const;

  inline Vector sqr() const;
  inline Vector cube() const;
  inline Vector inverse() const;

  inline bool isFinite() const;

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

  inline friend std::ostream& operator<<(std::ostream& os, const Vector& arg);
};

inline Vector::Vector()
{
}

inline Vector::Vector(double x, double y, double z) : data(x, y, z)
{
}

inline Vector::Vector(const Eigen::Vector3d& _data) : data(_data)
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

inline double Vector::min() const
{
  return data.minCoeff();
}

inline double Vector::max() const
{
  return data.maxCoeff();
}

inline double Vector::dot(const Vector& rhs) const
{
  return data.dot(rhs.data);
}

inline Vector Vector::hadamard(const Vector& rhs) const
{
  return Vector(data.cwiseProduct(rhs.data));
}

inline double Vector::argument(const Vector& rhs) const
{
  return ::acos(normalized().dot(rhs.normalized()));
}

inline bool Vector::isOrthogonal(const Vector& rhs) const
{
  return tobas_std::isClose(this->dot(rhs), 0.);
}

inline Vector Vector::clamp(const double& lb, const double& ub) const
{
  return Vector(data.cwiseMax(lb).cwiseMin(ub));
}

inline Vector Vector::clamp(const Vector& lb, const Vector& ub) const
{
  return Vector(data.cwiseMax(lb.data).cwiseMin(ub.data));
}

inline void Vector::setZero()
{
  data.setZero();
}

inline double Vector::norm() const
{
  return data.norm();
}

inline void Vector::normalize()
{
  assert(this->norm() > 0.);
  data.normalize();
}

inline Vector Vector::normalized() const
{
  assert(this->norm() > 0.);
  return Vector(data.normalized());
}

inline Vector Vector::sqr() const
{
  return Vector(data.cwiseAbs2());
}

inline Vector Vector::cube() const
{
  return Vector(data.cwiseProduct(data).cwiseProduct(data));
}

inline Vector Vector::inverse() const
{
  assert(x() != 0 && y() != 0 && z() != 0);
  return Vector(data.cwiseInverse());
}

bool Vector::isFinite() const
{
  return eigen::isFinite(data);
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

inline std::ostream& operator<<(std::ostream& os, const Vector& arg)
{
  os << arg.data.transpose();
  return os;
}
}  // namespace kdl
