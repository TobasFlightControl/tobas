#pragma once

#include <dh_eigen_tools/typedef.hpp>

#include "./frames.hpp"
#include "./jntarray.hpp"
#include "./segmentjacobian.hpp"

namespace KDL
{
class Jacobian;
using JacobianMap = std::map<std::string, Jacobian>;

class Jacobian
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 6, Eigen::Dynamic> data;

  inline explicit Jacobian();
  inline explicit Jacobian(size_t nj);

  /// Allocates memory for new size (can break realtime behavior)
  inline void resize(size_t nj);

  inline size_t rows() const;
  inline size_t columns() const;

  inline SegmentJacobian getColumn(size_t i) const;
  inline void setColumn(size_t i, const SegmentJacobian& jac);

  inline double operator()(size_t i, size_t j) const;
  inline double& operator()(size_t i, size_t j);

  inline Twist operator*(const JntArray& rhs) const;

  inline friend void setToZero(Jacobian& jac);

  void changeRefPoint(const Vector& base_AB);
  void changeBase(const Rotation& rot);
  void changeRefFrame(const Frame& frame);
  friend bool changeRefPoint(const Jacobian& src1, const Vector& base_AB, Jacobian& dest);
  friend bool changeBase(const Jacobian& src1, const Rotation& rot, Jacobian& dest);
  friend bool changeRefFrame(const Jacobian& src1, const Frame& frame, Jacobian& dest);
};

inline Jacobian::Jacobian()
{
}

inline Jacobian::Jacobian(size_t nj) : data(6, nj)
{
  data.setZero();
}

inline void Jacobian::resize(size_t nj)
{
  data.conservativeResize(Eigen::NoChange, nj);
}

inline size_t Jacobian::rows() const
{
  return static_cast<size_t>(data.rows());
}

inline size_t Jacobian::columns() const
{
  return static_cast<size_t>(data.cols());
}

inline SegmentJacobian Jacobian::getColumn(size_t i) const
{
  return SegmentJacobian(
    Vector(data(0, i), data(1, i), data(2, i)), Vector(data(3, i), data(4, i), data(5, i)));
}

inline void Jacobian::setColumn(size_t i, const SegmentJacobian& jac)
{
  data.col(i).head<3>() = jac.linear.data;
  data.col(i).tail<3>() = jac.angular.data;
}

inline double Jacobian::operator()(size_t i, size_t j) const
{
  return data(i, j);
}

inline double& Jacobian::operator()(size_t i, size_t j)
{
  return data(i, j);
}

inline Twist Jacobian::operator*(const JntArray& rhs) const
{
  const Eigen::Vector6d t = data * rhs.data;
  return Twist(Vector(t.head<3>()), Vector(t.tail<3>()));
}

inline void setToZero(Jacobian& jac)
{
  jac.data.setZero();
}
}  // namespace KDL
