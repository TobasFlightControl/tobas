#pragma once

#include "./frames.hpp"
#include "./jacobian.hpp"
#include "./jntarray.hpp"

namespace tobas_kdl
{
/**
 * @brief This class represents an fixed nj matrix containing
 * the Joint-Space Inertia Matrix of a tobas_kdl::Chain.
 *
 * \warning An object constructed with the default constructor provides
 * a valid, but inert, object. Many of the member functions will do
 * the correct thing and have no affect on this object, but some
 * member functions can _NOT_ deal with an inert/empty object. These
 * functions will assert() and exit the program instead. The intended use
 * case for the default constructor (in an RTT/OCL setting) is outlined in
 * code below - the default constructor plus the resize() function allow
 * use of JntSpaceInertiaMatrix objects whose nj is set within a configureHook() call
 * (typically based on a nj determined from a property).
 */
class JntSpaceInertiaMatrix
{
public:
  Eigen::MatrixXd data;

  /** Construct with _no_ data array
   * @post NULL == data
   * @post 0 == rows()
   * @warning use of an object constructed like this, without
   * a resize() first, may result in program exit! See class
   * documentation.
   */
  inline explicit JntSpaceInertiaMatrix();

  /**
   * Constructor of the Joint-Space Inertia Matrix
   *
   * @param nj of the matrix, this cannot be changed
   * afterwards. Size rows and nj columns.
   * @pre 0 < nj
   * @post NULL != data
   * @post 0 < rows()
   * @post all elements in data have 0 value
   */
  inline explicit JntSpaceInertiaMatrix(int nj);

  /**
   * Resize the array
   * @warning This causes a dynamic allocation (and potentially
   * also a dynamic deallocation). This _will_ negatively affect
   * real-time performance!
   *
   * @post nj == rows()
   * @post NULL != data
   * @post all elements in data have 0 value
   */
  inline void resize(size_t nj);

  /**
   * get_item operator for the joint matrix
   *
   * @return the joint value at position i, starting from 0
   * @pre 0 != nj (ie non-default constructor or resize() called)
   */
  inline double operator()(size_t i, size_t j) const;

  /**
   * set_item operator
   *
   * @return reference to the joint value at position i,starting
   *from zero.
   * @pre 0 != nj (ie non-default constructor or resize() called)
   */
  inline double& operator()(size_t i, size_t j);

  /**
   * Returns the number of rows and columns of the matrix.
   */
  inline size_t rows() const;

  /**
   * Returns the number of columns of the matrix.
   */
  inline size_t columns() const;

  inline friend void Add(
    const JntSpaceInertiaMatrix& src1,
    const JntSpaceInertiaMatrix& src2,
    JntSpaceInertiaMatrix& dest);
  inline friend void Subtract(
    const JntSpaceInertiaMatrix& src1,
    const JntSpaceInertiaMatrix& src2,
    JntSpaceInertiaMatrix& dest);
  inline friend void
  Multiply(const JntSpaceInertiaMatrix& src, const double& factor, JntSpaceInertiaMatrix& dest);
  inline friend void
  Divide(const JntSpaceInertiaMatrix& src, const double& factor, JntSpaceInertiaMatrix& dest);
  inline friend void
  Multiply(const JntSpaceInertiaMatrix& src, const JntArray& vec, JntArray& dest);
  inline friend void setToZero(JntSpaceInertiaMatrix& matrix);
};

inline JntSpaceInertiaMatrix::JntSpaceInertiaMatrix()
{
}

inline JntSpaceInertiaMatrix::JntSpaceInertiaMatrix(int nj) : data(nj, nj)
{
  data.setZero();
}

inline void JntSpaceInertiaMatrix::resize(size_t nj)
{
  data.resize(nj, nj);
}

inline double JntSpaceInertiaMatrix::operator()(size_t i, size_t j) const
{
  return data(i, j);
}

inline double& JntSpaceInertiaMatrix::operator()(size_t i, size_t j)
{
  return data(i, j);
}

inline size_t JntSpaceInertiaMatrix::rows() const
{
  return static_cast<size_t>(data.rows());
}

inline size_t JntSpaceInertiaMatrix::columns() const
{
  return static_cast<size_t>(data.cols());
}

inline void Add(
  const JntSpaceInertiaMatrix& src1,
  const JntSpaceInertiaMatrix& src2,
  JntSpaceInertiaMatrix& dest)
{
  dest.data = src1.data + src2.data;
}

inline void Subtract(
  const JntSpaceInertiaMatrix& src1,
  const JntSpaceInertiaMatrix& src2,
  JntSpaceInertiaMatrix& dest)
{
  dest.data = src1.data - src2.data;
}

inline void
Multiply(const JntSpaceInertiaMatrix& src, const double& factor, JntSpaceInertiaMatrix& dest)
{
  dest.data = factor * src.data;
}

inline void
Divide(const JntSpaceInertiaMatrix& src, const double& factor, JntSpaceInertiaMatrix& dest)
{
  dest.data = src.data / factor;
}

inline void Multiply(const JntSpaceInertiaMatrix& src, const JntArray& vec, JntArray& dest)
{
  dest.data = src.data.lazyProduct(vec.data);
}

inline void setToZero(JntSpaceInertiaMatrix& mat)
{
  mat.data.setZero();
}
}  // namespace tobas_kdl
