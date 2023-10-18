#pragma once

#include "./vector3.hpp"
#include "./vector2.hpp"

namespace tobas_mr_arducopter
{
template <typename T>
class Vector3;

// 3x3 matrix with elements of type T
template <typename T>
class Matrix3
{
public:
  // Vectors comprising the rows of the matrix
  Vector3<T> a, b, c;

  // trivial ctor
  // note that the Vector3 ctor will zero the vector elements
  constexpr Matrix3<T>()
  {
  }

  // setting ctor
  constexpr Matrix3<T>(const Vector3<T>& a0, const Vector3<T>& b0, const Vector3<T>& c0)
    : a(a0), b(b0), c(c0)
  {
  }

  // setting ctor
  constexpr Matrix3<T>(
    const T ax,
    const T ay,
    const T az,
    const T bx,
    const T by,
    const T bz,
    const T cx,
    const T cy,
    const T cz)
    : a(ax, ay, az), b(bx, by, bz), c(cx, cy, cz)
  {
  }

  // function call operator
  void operator()(const Vector3<T>& a0, const Vector3<T>& b0, const Vector3<T>& c0)
  {
    a = a0;
    b = b0;
    c = c0;
  }

  // test for equality
  bool operator==(const Matrix3<T>& m)
  {
    return (a == m.a && b == m.b && c == m.c);
  }

  // test for inequality
  bool operator!=(const Matrix3<T>& m)
  {
    return (a != m.a || b != m.b || c != m.c);
  }

  // negation
  Matrix3<T> operator-(void) const
  {
    return Matrix3<T>(-a, -b, -c);
  }

  // addition
  Matrix3<T> operator+(const Matrix3<T>& m) const
  {
    return Matrix3<T>(a + m.a, b + m.b, c + m.c);
  }
  Matrix3<T>& operator+=(const Matrix3<T>& m)
  {
    return *this = *this + m;
  }

  // subtraction
  Matrix3<T> operator-(const Matrix3<T>& m) const
  {
    return Matrix3<T>(a - m.a, b - m.b, c - m.c);
  }
  Matrix3<T>& operator-=(const Matrix3<T>& m)
  {
    return *this = *this - m;
  }

  // uniform scaling
  Matrix3<T> operator*(const T num) const
  {
    return Matrix3<T>(a * num, b * num, c * num);
  }
  Matrix3<T>& operator*=(const T num)
  {
    return *this = *this * num;
  }
  Matrix3<T> operator/(const T num) const
  {
    return Matrix3<T>(a / num, b / num, c / num);
  }
  Matrix3<T>& operator/=(const T num)
  {
    return *this = *this / num;
  }

  // allow a Matrix3 to be used as an array of vectors, 0 indexed
  Vector3<T>& operator[](uint8_t i)
  {
    Vector3<T>* _v = &a;
#if MATH_CHECK_INDEXES
    assert(i >= 0 && i < 3);
#endif
    return _v[i];
  }

  const Vector3<T>& operator[](uint8_t i) const
  {
    const Vector3<T>* _v = &a;
#if MATH_CHECK_INDEXES
    assert(i >= 0 && i < 3);
#endif
    return _v[i];
  }

  // multiplication by a vector
  Vector3<T> operator*(const Vector3<T>& v) const;

  // multiplication of transpose by a vector
  Vector3<T> mul_transpose(const Vector3<T>& v) const;

  // multiplication by a vector giving a Vector2 result (XY components)
  Vector2<T> mulXY(const Vector3<T>& v) const;

  // extract x column
  Vector3<T> colx(void) const
  {
    return Vector3<T>(a.x, b.x, c.x);
  }

  // extract y column
  Vector3<T> coly(void) const
  {
    return Vector3<T>(a.y, b.y, c.y);
  }

  // extract z column
  Vector3<T> colz(void) const
  {
    return Vector3<T>(a.z, b.z, c.z);
  }

  // multiplication by another Matrix3<T>
  Matrix3<T> operator*(const Matrix3<T>& m) const;

  Matrix3<T>& operator*=(const Matrix3<T>& m)
  {
    return *this = *this * m;
  }

  // transpose the matrix
  Matrix3<T> transposed(void) const;

  void transpose(void)
  {
    *this = transposed();
  }

  /**
   * Calculate the determinant of this matrix.
   *
   * @return The value of the determinant.
   */
  T det() const;

  /**
   * Calculate the inverse of this matrix.
   *
   * @param inv[in] Where to store the result.
   *
   * @return If this matrix is invertible, then true is returned. Otherwise,
   * \p inv is unmodified and false is returned.
   */
  bool inverse(Matrix3<T>& inv) const;

  /**
   * Invert this matrix if it is invertible.
   *
   * @return Return true if this matrix could be successfully inverted and
   * false otherwise.
   */
  bool invert();

  // zero the matrix
  void zero(void);

  // setup the identity matrix
  void identity(void)
  {
    a.x = b.y = c.z = 1;
    a.y = a.z = 0;
    b.x = b.z = 0;
    c.x = c.y = 0;
  }

  // check if any elements are NAN
  bool is_nan(void)
  {
    return a.is_nan() || b.is_nan() || c.is_nan();
  }

  // create a rotation matrix from Euler angles
  void from_euler(T roll, T pitch, T yaw);

  // create eulers from a rotation matrix.
  // roll is from -Pi to Pi
  // pitch is from -Pi/2 to Pi/2
  // yaw is from -Pi to Pi
  void to_euler(T* roll, T* pitch, T* yaw) const;

  // create matrix from rotation enum
  void from_rotation(Rotation rotation);

  /*
    calculate Euler angles (312 convention) for the matrix.
    See http://www.atacolorado.com/eulersequences.doc
    vector is returned in r, p, y order
  */
  Vector3<T> to_euler312() const;

  /*
    fill the matrix from Euler angles in radians in 312 convention
  */
  void from_euler312(T roll, T pitch, T yaw);

  // apply an additional rotation from a body frame gyro vector
  // to a rotation matrix.
  void rotate(const Vector3<T>& g);

  // create rotation matrix for rotation about the vector v by angle theta
  // See: https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
  // "Rotation matrix from axis and angle"
  void from_axis_angle(const Vector3<T>& v, T theta);

  // normalize a rotation matrix
  void normalize(void);

  // double/float conversion
  Matrix3<double> todouble(void) const
  {
    return Matrix3<double>(a.todouble(), b.todouble(), c.todouble());
  }
  Matrix3<float> tofloat(void) const
  {
    return Matrix3<float>(a.tofloat(), b.tofloat(), c.tofloat());
  }
};

typedef Matrix3<int16_t> Matrix3i;
typedef Matrix3<uint16_t> Matrix3ui;
typedef Matrix3<int32_t> Matrix3l;
typedef Matrix3<uint32_t> Matrix3ul;
typedef Matrix3<float> Matrix3f;
typedef Matrix3<double> Matrix3d;

// create a rotation matrix given some euler angles
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
template <typename T>
void Matrix3<T>::from_euler(T roll, T pitch, T yaw)
{
  const T cp = cosF(pitch);
  const T sp = sinF(pitch);
  const T sr = sinF(roll);
  const T cr = cosF(roll);
  const T sy = sinF(yaw);
  const T cy = cosF(yaw);

  a.x = cp * cy;
  a.y = (sr * sp * cy) - (cr * sy);
  a.z = (cr * sp * cy) + (sr * sy);
  b.x = cp * sy;
  b.y = (sr * sp * sy) + (cr * cy);
  b.z = (cr * sp * sy) - (sr * cy);
  c.x = -sp;
  c.y = sr * cp;
  c.z = cr * cp;
}

// calculate euler angles from a rotation matrix
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
template <typename T>
void Matrix3<T>::to_euler(T* roll, T* pitch, T* yaw) const
{
  if (pitch != nullptr)
  {
    *pitch = -safe_asin(c.x);
  }
  if (roll != nullptr)
  {
    *roll = atan2F(c.y, c.z);
  }
  if (yaw != nullptr)
  {
    *yaw = atan2F(b.x, a.x);
  }
}

template <typename T>
void Matrix3<T>::from_rotation(Rotation rotation)
{
  (*this).a = { 1, 0, 0 };
  (*this).b = { 0, 1, 0 };
  (*this).c = { 0, 0, 1 };

  (*this).a.rotate(rotation);
  (*this).b.rotate(rotation);
  (*this).c.rotate(rotation);
  (*this).transpose();
}

/*
  calculate Euler angles (312 convention) for the matrix.
  See http://www.atacolorado.com/eulersequences.doc
  vector is returned in r, p, y order
*/
template <typename T>
Vector3<T> Matrix3<T>::to_euler312() const
{
  return Vector3<T>(asinF(c.y), atan2F(-c.x, c.z), atan2F(-a.y, b.y));
}

/*
  fill the matrix from Euler angles in radians in 312 convention
*/
template <typename T>
void Matrix3<T>::from_euler312(T roll, T pitch, T yaw)
{
  const T c3 = cosF(pitch);
  const T s3 = sinF(pitch);
  const T s2 = sinF(roll);
  const T c2 = cosF(roll);
  const T s1 = sinF(yaw);
  const T c1 = cosF(yaw);

  a.x = c1 * c3 - s1 * s2 * s3;
  b.y = c1 * c2;
  c.z = c3 * c2;
  a.y = -c2 * s1;
  a.z = s3 * c1 + c3 * s2 * s1;
  b.x = c3 * s1 + s3 * s2 * c1;
  b.z = s1 * s3 - s2 * c1 * c3;
  c.x = -s3 * c2;
  c.y = s2;
}

// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
template <typename T>
void Matrix3<T>::rotate(const Vector3<T>& g)
{
  (*this) += Matrix3<T>{ a.y * g.z - a.z * g.y, a.z * g.x - a.x * g.z, a.x * g.y - a.y * g.x,
                         b.y * g.z - b.z * g.y, b.z * g.x - b.x * g.z, b.x * g.y - b.y * g.x,
                         c.y * g.z - c.z * g.y, c.z * g.x - c.x * g.z, c.x * g.y - c.y * g.x };
}

/*
  re-normalise a rotation matrix
*/
template <typename T>
void Matrix3<T>::normalize(void)
{
  const T error = a * b;
  const Vector3<T> t0 = a - (b * (0.5f * error));
  const Vector3<T> t1 = b - (a * (0.5f * error));
  const Vector3<T> t2 = t0 % t1;
  a = t0 * (1.0f / t0.length());
  b = t1 * (1.0f / t1.length());
  c = t2 * (1.0f / t2.length());
}

// multiplication by a vector
template <typename T>
Vector3<T> Matrix3<T>::operator*(const Vector3<T>& v) const
{
  return Vector3<T>(
    a.x * v.x + a.y * v.y + a.z * v.z, b.x * v.x + b.y * v.y + b.z * v.z,
    c.x * v.x + c.y * v.y + c.z * v.z);
}

// multiplication by a vector, extracting only the xy components
template <typename T>
Vector2<T> Matrix3<T>::mulXY(const Vector3<T>& v) const
{
  return Vector2<T>(a.x * v.x + a.y * v.y + a.z * v.z, b.x * v.x + b.y * v.y + b.z * v.z);
}

// multiplication of transpose by a vector
template <typename T>
Vector3<T> Matrix3<T>::mul_transpose(const Vector3<T>& v) const
{
  return Vector3<T>(
    a.x * v.x + b.x * v.y + c.x * v.z, a.y * v.x + b.y * v.y + c.y * v.z,
    a.z * v.x + b.z * v.y + c.z * v.z);
}

// multiplication by another Matrix3<T>
template <typename T>
Matrix3<T> Matrix3<T>::operator*(const Matrix3<T>& m) const
{
  Matrix3<T> temp(
    Vector3<T>(
      a.x * m.a.x + a.y * m.b.x + a.z * m.c.x, a.x * m.a.y + a.y * m.b.y + a.z * m.c.y,
      a.x * m.a.z + a.y * m.b.z + a.z * m.c.z),
    Vector3<T>(
      b.x * m.a.x + b.y * m.b.x + b.z * m.c.x, b.x * m.a.y + b.y * m.b.y + b.z * m.c.y,
      b.x * m.a.z + b.y * m.b.z + b.z * m.c.z),
    Vector3<T>(
      c.x * m.a.x + c.y * m.b.x + c.z * m.c.x, c.x * m.a.y + c.y * m.b.y + c.z * m.c.y,
      c.x * m.a.z + c.y * m.b.z + c.z * m.c.z));
  return temp;
}

template <typename T>
Matrix3<T> Matrix3<T>::transposed(void) const
{
  return Matrix3<T>(
    Vector3<T>(a.x, b.x, c.x), Vector3<T>(a.y, b.y, c.y), Vector3<T>(a.z, b.z, c.z));
}

template <typename T>
T Matrix3<T>::det() const
{
  return a.x * (b.y * c.z - b.z * c.y) + a.y * (b.z * c.x - b.x * c.z)
         + a.z * (b.x * c.y - b.y * c.x);
}

template <typename T>
bool Matrix3<T>::inverse(Matrix3<T>& inv) const
{
  const T d = det();

  if (is_zero(d))
  {
    return false;
  }

  inv.a.x = (b.y * c.z - c.y * b.z) / d;
  inv.a.y = (a.z * c.y - a.y * c.z) / d;
  inv.a.z = (a.y * b.z - a.z * b.y) / d;
  inv.b.x = (b.z * c.x - b.x * c.z) / d;
  inv.b.y = (a.x * c.z - a.z * c.x) / d;
  inv.b.z = (b.x * a.z - a.x * b.z) / d;
  inv.c.x = (b.x * c.y - c.x * b.y) / d;
  inv.c.y = (c.x * a.y - a.x * c.y) / d;
  inv.c.z = (a.x * b.y - b.x * a.y) / d;

  return true;
}

template <typename T>
bool Matrix3<T>::invert()
{
  Matrix3<T> inv;
  bool success = inverse(inv);
  if (success)
  {
    *this = inv;
  }
  return success;
}

template <typename T>
void Matrix3<T>::zero(void)
{
  a.x = a.y = a.z = 0;
  b.x = b.y = b.z = 0;
  c.x = c.y = c.z = 0;
}

// create rotation matrix for rotation about the vector v by angle theta
// See: http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
template <typename T>
void Matrix3<T>::from_axis_angle(const Vector3<T>& v, T theta)
{
  const T C = cosF(theta);
  const T S = sinF(theta);
  const T t = 1.0f - C;
  const Vector3<T> normv = v.normalized();
  const T x = normv.x;
  const T y = normv.y;
  const T z = normv.z;

  a.x = t * x * x + C;
  a.y = t * x * y - z * S;
  a.z = t * x * z + y * S;
  b.x = t * x * y + z * S;
  b.y = t * y * y + C;
  b.z = t * y * z - x * S;
  c.x = t * x * z - y * S;
  c.y = t * y * z + x * S;
  c.z = t * z * z + C;
}

// define for float and double
template class Matrix3<float>;
template class Matrix3<double>;
}  // namespace tobas_mr_arducopter
