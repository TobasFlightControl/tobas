#pragma once

#include "./frames.hpp"

namespace kdl
{
class JntArray
{
public:
  Eigen::VectorXd data;

  inline explicit JntArray();
  inline explicit JntArray(size_t nj);
  inline explicit JntArray(const Eigen::VectorXd& data);

  inline static JntArray Zero(size_t nj);
  inline static JntArray Constant(size_t nj, double value);

  inline void resize(size_t nj);

  inline double operator()(size_t i) const;
  inline double& operator()(size_t i);

  inline size_t size() const;
  inline size_t rows() const;

  inline double max() const;
  inline double min() const;

  inline JntArray max(double x);
  inline JntArray min(double x);

  inline JntArray hadamard(const JntArray& arg);

  inline JntArray operator+(const JntArray& rhs) const;
  inline JntArray operator-(const JntArray& rhs) const;
  inline JntArray operator*(const double& rhs) const;
  inline JntArray operator/(const double& rhs) const;
  inline JntArray& operator+=(const JntArray& rhs);
  inline JntArray& operator-=(const JntArray& rhs);
  inline JntArray& operator*=(const double& rhs);
  inline JntArray& operator/=(const double& rhs);

  inline friend JntArray operator+(const JntArray& arg);
  inline friend JntArray operator-(const JntArray& arg);
  inline friend JntArray operator*(const double& lhs, const JntArray& rhs);

  inline void setZero();
  inline friend void setToZero(JntArray& array);

  friend std::ostream& operator<<(std::ostream& os, const JntArray& arg);
};

inline JntArray::JntArray()
{
}

inline JntArray::JntArray(size_t nj) : data(nj)
{
}

inline JntArray::JntArray(const Eigen::VectorXd& data) : data(data)
{
}

inline JntArray JntArray::Zero(size_t nj)
{
  return JntArray(Eigen::VectorXd::Zero(nj));
}

inline JntArray JntArray::Constant(size_t nj, double value)
{
  return JntArray(Eigen::VectorXd::Constant(nj, value));
}

inline void JntArray::resize(size_t nj)
{
  data.conservativeResize(nj);
}

inline double JntArray::operator()(size_t i) const
{
  return data(i);
}

inline double& JntArray::operator()(size_t i)
{
  return data(i);
}

inline size_t JntArray::size() const
{
  return static_cast<size_t>(data.rows());
}

inline size_t JntArray::rows() const
{
  return size();
}

inline double JntArray::max() const
{
  return data.maxCoeff();
}

inline double JntArray::min() const
{
  return data.minCoeff();
}

inline JntArray JntArray::max(double x)
{
  return JntArray(data.cwiseMax(x));
}

inline JntArray JntArray::min(double x)
{
  return JntArray(data.cwiseMin(x));
}

inline JntArray JntArray::hadamard(const JntArray& arg)
{
  assert(rows() == arg.rows());
  return JntArray(data.cwiseProduct(arg.data));
}

inline JntArray JntArray::operator+(const JntArray& rhs) const
{
  assert(rows() == rhs.rows());
  return JntArray(data + rhs.data);
}

inline JntArray JntArray::operator-(const JntArray& rhs) const
{
  assert(rows() == rhs.rows());
  return JntArray(data - rhs.data);
}

inline JntArray JntArray::operator*(const double& rhs) const
{
  return JntArray(data * rhs);
}

inline JntArray JntArray::operator/(const double& rhs) const
{
  assert(rhs != 0);
  return JntArray(data / rhs);
}

inline JntArray& JntArray::operator+=(const JntArray& rhs)
{
  assert(rows() == rhs.rows());
  data += rhs.data;
  return *this;
}

inline JntArray& JntArray::operator-=(const JntArray& rhs)
{
  assert(rows() == rhs.rows());
  data -= rhs.data;
  return *this;
}

inline JntArray& JntArray::operator*=(const double& rhs)
{
  data *= rhs;
  return *this;
}

inline JntArray& JntArray::operator/=(const double& rhs)
{
  assert(rhs != 0);
  data /= rhs;
  return *this;
}

inline JntArray operator+(const JntArray& arg)
{
  return arg;
}

inline JntArray operator-(const JntArray& arg)
{
  return JntArray(-arg.data);
}

inline JntArray operator*(const double& lhs, const JntArray& rhs)
{
  return JntArray(lhs * rhs.data);
}

void JntArray::setZero()
{
  data.setZero();
}

inline void setToZero(JntArray& array)
{
  array.data.setZero();
}
}  // namespace kdl
