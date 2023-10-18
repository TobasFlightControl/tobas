#pragma once

#include <cmath>
#include <cinttypes>

namespace tobas_mr_arducopter
{
template <typename T>
struct Vector2
{
  T x, y;

  // trivial ctor
  constexpr Vector2<T>() : x(0), y(0)
  {
  }

  // setting ctor
  constexpr Vector2<T>(const T x0, const T y0) : x(x0), y(y0)
  {
  }

  // test for equality
  bool operator==(const Vector2<T>& v) const;

  // test for inequality
  bool operator!=(const Vector2<T>& v) const;

  // negation
  Vector2<T> operator-(void) const;

  // addition
  Vector2<T> operator+(const Vector2<T>& v) const;

  // subtraction
  Vector2<T> operator-(const Vector2<T>& v) const;

  // uniform scaling
  Vector2<T> operator*(const T num) const;

  // uniform scaling
  Vector2<T> operator/(const T num) const;

  // addition
  Vector2<T>& operator+=(const Vector2<T>& v);

  // subtraction
  Vector2<T>& operator-=(const Vector2<T>& v);

  // uniform scaling
  Vector2<T>& operator*=(const T num);

  // uniform scaling
  Vector2<T>& operator/=(const T num);

  // dot product
  T operator*(const Vector2<T>& v) const;

  // dot product (same as above but a more easily understood name)
  T dot(const Vector2<T>& v) const
  {
    return *this * v;
  }

  // cross product
  T operator%(const Vector2<T>& v) const;

  // computes the angle between this vector and another vector
  // returns 0 if the vectors are parallel, and M_PI if they are antiparallel
  T angle(const Vector2<T>& v2) const;

  // computes the angle of this vector in radians, from 0 to 2pi,
  // from a unit vector(1,0); a (1,1) vector's angle is +M_PI/4
  T angle(void) const;

  // check if any elements are NAN
  bool is_nan(void) const;

  // check if any elements are infinity
  bool is_inf(void) const;

  // check if all elements are zero
  bool is_zero(void) const
  {
    return x == 0 && y == 0;
  }

  // allow a vector2 to be used as an array, 0 indexed
  T& operator[](uint8_t i)
  {
    T* _v = &x;
    return _v[i];
  }

  const T& operator[](uint8_t i) const
  {
    const T* _v = &x;
    return _v[i];
  }

  // zero the vector
  void zero()
  {
    x = y = 0;
  }

  // gets the length of this vector squared
  T length_squared() const;

  // gets the length of this vector
  T length(void) const;

  // limit vector to a given length. returns true if vector was limited
  bool limit_length(T max_length);

  // normalizes this vector
  void normalize();

  // returns the normalized vector
  Vector2<T> normalized() const;

  // reflects this vector about n
  void reflect(const Vector2<T>& n);

  // projects this vector onto v
  void project(const Vector2<T>& v);

  // returns this vector projected onto v
  Vector2<T> projected(const Vector2<T>& v) const;

  // adjust position by a given bearing (in degrees) and distance
  void offset_bearing(T bearing, T distance);

  // rotate vector by angle in radians
  void rotate(T angle_rad);

  /*
    conversion to/from double
   */
  Vector2<float> tofloat() const
  {
    return Vector2<float>{ float(x), float(y) };
  }
  Vector2<double> todouble() const
  {
    return Vector2<double>{ x, y };
  }

  // given a position p1 and a velocity v1 produce a vector
  // perpendicular to v1 maximising distance from p1
  static Vector2<T> perpendicular(const Vector2<T>& pos_delta, const Vector2<T>& v1);

  /*
   * Returns the point closest to p on the line segment (v,w).
   *
   * Comments and implementation taken from
   * http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
   */
  static Vector2<T> closest_point(const Vector2<T>& p, const Vector2<T>& v, const Vector2<T>& w);

  /*
   * Returns the point closest to p on the line segment (0,w).
   *
   * this is a simplification of closest point with a general segment, with v=(0,0)
   */
  static Vector2<T> closest_point(const Vector2<T>& p, const Vector2<T>& w);

  // w1 and w2 define a line segment
  // p is a point
  // returns the square of the closest distance between the line segment and the point
  static T closest_distance_between_line_and_point_squared(
    const Vector2<T>& w1,
    const Vector2<T>& w2,
    const Vector2<T>& p);

  // w1 and w2 define a line segment
  // p is a point
  // returns the closest distance between the line segment and the point
  static T closest_distance_between_line_and_point(
    const Vector2<T>& w1,
    const Vector2<T>& w2,
    const Vector2<T>& p);

  // a1->a2 and b2->v2 define two line segments
  // returns the square of the closest distance between the two line segments
  static T closest_distance_between_lines_squared(
    const Vector2<T>& a1,
    const Vector2<T>& a2,
    const Vector2<T>& b1,
    const Vector2<T>& b2);

  // w defines a line segment from the origin
  // p is a point
  // returns the square of the closest distance between the radial and the point
  static T
  closest_distance_between_radial_and_point_squared(const Vector2<T>& w, const Vector2<T>& p);

  // w defines a line segment from the origin
  // p is a point
  // returns the closest distance between the radial and the point
  static T closest_distance_between_radial_and_point(const Vector2<T>& w, const Vector2<T>& p);

  // find the intersection between two line segments
  // returns true if they intersect, false if they do not
  // the point of intersection is returned in the intersection argument
  static bool segment_intersection(
    const Vector2<T>& seg1_start,
    const Vector2<T>& seg1_end,
    const Vector2<T>& seg2_start,
    const Vector2<T>& seg2_end,
    Vector2<T>& intersection);

  // find the intersection between a line segment and a circle
  // returns true if they intersect and intersection argument is updated with intersection closest
  // to seg_start
  static bool circle_segment_intersection(
    const Vector2<T>& seg_start,
    const Vector2<T>& seg_end,
    const Vector2<T>& circle_center,
    T radius,
    Vector2<T>& intersection);

  // check if a point falls on the line segment from seg_start to seg_end
  static bool
  point_on_segment(const Vector2<T>& point, const Vector2<T>& seg_start, const Vector2<T>& seg_end)
  {
    const T expected_run = seg_end.x - seg_start.x;
    const T intersection_run = point.x - seg_start.x;
    // check slopes are identical:
    if (::is_zero(expected_run))
    {
      if (fabsF(intersection_run) > FLT_EPSILON)
      {
        return false;
      }
    }
    else
    {
      const T expected_slope = (seg_end.y - seg_start.y) / expected_run;
      const T intersection_slope = (point.y - seg_start.y) / intersection_run;
      if (fabsF(expected_slope - intersection_slope) > FLT_EPSILON)
      {
        return false;
      }
    }
    // check for presence in bounding box
    if (seg_start.x < seg_end.x)
    {
      if (point.x < seg_start.x || point.x > seg_end.x)
      {
        return false;
      }
    }
    else
    {
      if (point.x < seg_end.x || point.x > seg_start.x)
      {
        return false;
      }
    }
    if (seg_start.y < seg_end.y)
    {
      if (point.y < seg_start.y || point.y > seg_end.y)
      {
        return false;
      }
    }
    else
    {
      if (point.y < seg_end.y || point.y > seg_start.y)
      {
        return false;
      }
    }
    return true;
  }
};

// check if all elements are zero
template <>
inline bool Vector2<float>::is_zero(void) const
{
  return x == 0 && y == 0;
}

template <>
inline bool Vector2<double>::is_zero(void) const
{
  return x == 0 && y == 0;
}

typedef Vector2<int16_t> Vector2i;
typedef Vector2<uint16_t> Vector2ui;
typedef Vector2<int32_t> Vector2l;
typedef Vector2<uint32_t> Vector2ul;
typedef Vector2<float> Vector2f;
typedef Vector2<double> Vector2d;

template <typename T>
T Vector2<T>::length_squared() const
{
  return (T)(x * x + y * y);
}

template <typename T>
T Vector2<T>::length(void) const
{
  return norm(x, y);
}

// limit vector to a given length. returns true if vector was limited
template <typename T>
bool Vector2<T>::limit_length(T max_length)
{
  const T len = length();
  if ((len > max_length) && is_positive(len))
  {
    x *= (max_length / len);
    y *= (max_length / len);
    return true;
  }
  return false;
}

// dot product
template <typename T>
T Vector2<T>::operator*(const Vector2<T>& v) const
{
  return x * v.x + y * v.y;
}

// cross product
template <typename T>
T Vector2<T>::operator%(const Vector2<T>& v) const
{
  return x * v.y - y * v.x;
}

template <typename T>
Vector2<T>& Vector2<T>::operator*=(const T num)
{
  x *= num;
  y *= num;
  return *this;
}

template <typename T>
Vector2<T>& Vector2<T>::operator/=(const T num)
{
  x /= num;
  y /= num;
  return *this;
}

template <typename T>
Vector2<T>& Vector2<T>::operator-=(const Vector2<T>& v)
{
  x -= v.x;
  y -= v.y;
  return *this;
}

template <typename T>
bool Vector2<T>::is_nan(void) const
{
  return isnan(x) || isnan(y);
}

template <typename T>
bool Vector2<T>::is_inf(void) const
{
  return isinf(x) || isinf(y);
}

template <typename T>
Vector2<T>& Vector2<T>::operator+=(const Vector2<T>& v)
{
  x += v.x;
  y += v.y;
  return *this;
}

template <typename T>
Vector2<T> Vector2<T>::operator/(const T num) const
{
  return Vector2<T>(x / num, y / num);
}

template <typename T>
Vector2<T> Vector2<T>::operator*(const T num) const
{
  return Vector2<T>(x * num, y * num);
}

template <typename T>
Vector2<T> Vector2<T>::operator-(const Vector2<T>& v) const
{
  return Vector2<T>(x - v.x, y - v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator+(const Vector2<T>& v) const
{
  return Vector2<T>(x + v.x, y + v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator-(void) const
{
  return Vector2<T>(-x, -y);
}

template <typename T>
bool Vector2<T>::operator==(const Vector2<T>& v) const
{
  return (is_equal(x, v.x) && is_equal(y, v.y));
}

template <typename T>
bool Vector2<T>::operator!=(const Vector2<T>& v) const
{
  return (!is_equal(x, v.x) || !is_equal(y, v.y));
}

template <typename T>
T Vector2<T>::angle(const Vector2<T>& v2) const
{
  const T len = this->length() * v2.length();
  if (len <= 0)
  {
    return 0.0f;
  }
  const T cosv = ((*this) * v2) / len;
  if (cosv >= 1)
  {
    return 0.0f;
  }
  if (cosv <= -1)
  {
    return M_PI;
  }
  return acosF(cosv);
}

template <typename T>
T Vector2<T>::angle(void) const
{
  return M_PI_2 + atan2F(-x, y);
}

// find the intersection between two line segments
// returns true if they intersect, false if they do not
// the point of intersection is returned in the intersection argument
template <typename T>
bool Vector2<T>::segment_intersection(
  const Vector2<T>& seg1_start,
  const Vector2<T>& seg1_end,
  const Vector2<T>& seg2_start,
  const Vector2<T>& seg2_end,
  Vector2<T>& intersection)
{
  // implementation borrowed from
  // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
  const Vector2<T> r1 = seg1_end - seg1_start;
  const Vector2<T> r2 = seg2_end - seg2_start;
  const Vector2<T> ss2_ss1 = seg2_start - seg1_start;
  const T r1xr2 = r1 % r2;
  const T q_pxr = ss2_ss1 % r1;
  if (::is_zero(r1xr2))
  {
    // either collinear or parallel and non-intersecting
    return false;
  }
  else
  {
    // t = (q - p) * s / (r * s)
    // u = (q - p) * r / (r * s)
    const T t = (ss2_ss1 % r2) / r1xr2;
    const T u = q_pxr / r1xr2;
    if ((u >= 0) && (u <= 1) && (t >= 0) && (t <= 1))
    {
      // lines intersect
      // t can be any non-negative value because (p, p + r) is a ray
      // u must be between 0 and 1 because (q, q + s) is a line segment
      intersection = seg1_start + (r1 * t);
      return true;
    }
    else
    {
      // non-parallel and non-intersecting
      return false;
    }
  }
}

// find the intersection between a line segment and a circle
// returns true if they intersect and intersection argument is updated with intersection closest to
// seg_start solution adapted from
// http://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
template <typename T>
bool Vector2<T>::circle_segment_intersection(
  const Vector2<T>& seg_start,
  const Vector2<T>& seg_end,
  const Vector2<T>& circle_center,
  T radius,
  Vector2<T>& intersection)
{
  // calculate segment start and end as offsets from circle's center
  const Vector2<T> seg_start_local = seg_start - circle_center;

  // calculate vector from start to end
  const Vector2<T> seg_end_minus_start = seg_end - seg_start;

  const T a = sq(seg_end_minus_start.x) + sq(seg_end_minus_start.y);
  const T b =
    2 * ((seg_end_minus_start.x * seg_start_local.x) + (seg_end_minus_start.y * seg_start_local.y));
  const T c = sq(seg_start_local.x) + sq(seg_start_local.y) - sq(radius);

  // check for invalid data
  if (::is_zero(a) || isnan(a) || isnan(b) || isnan(c))
  {
    return false;
  }

  const T delta = sq(b) - (4.0f * a * c);

  if (isnan(delta))
  {
    return false;
  }

  // check for invalid delta (i.e. discriminant)
  if (delta < 0.0f)
  {
    return false;
  }

  const T delta_sqrt = sqrtF(delta);
  const T t1 = (-b + delta_sqrt) / (2.0f * a);
  const T t2 = (-b - delta_sqrt) / (2.0f * a);

  // Three hit cases:
  //          -o->             --|-->  |            |  --|->
  // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

  // Three miss cases:
  //       ->  o                     o ->              | -> |
  // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

  // intersection = new Vector3(E.x + t1 * d.x, secondPoint.y, E.y + t1 * d.y);
  //   intersection.x = seg_start.x + t1 * seg_end_minus_start.x;
  //   intersection.y = seg_start.y + t1 * seg_end_minus_start.y;

  if ((t1 >= 0.0f) && (t1 <= 1.0f))
  {
    // t1 is the intersection, and it is closer than t2 (since t1 uses -b - discriminant)
    // Impale, Poke
    intersection = seg_start + (seg_end_minus_start * t1);
    return true;
  }

  // here t1 did not intersect so we are either started inside the sphere or completely past it
  if ((t2 >= 0.0f) && (t2 <= 1.0f))
  {
    // ExitWound
    intersection = seg_start + (seg_end_minus_start * t2);
    return true;
  }

  // no intersection: FallShort, Past or CompletelyInside
  return false;
}

// normalizes this vector
template <typename T>
void Vector2<T>::normalize()
{
  *this /= length();
}

// returns the normalized vector
template <typename T>
Vector2<T> Vector2<T>::normalized() const
{
  return *this / length();
}

// reflects this vector about n
template <typename T>
void Vector2<T>::reflect(const Vector2<T>& n)
{
  const Vector2<T> orig(*this);
  project(n);
  *this = *this * 2 - orig;
}

// projects this vector onto v
template <typename T>
void Vector2<T>::project(const Vector2<T>& v)
{
  *this = v * (*this * v) / (v * v);
}

// returns this vector projected onto v
template <typename T>
Vector2<T> Vector2<T>::projected(const Vector2<T>& v) const
{
  return v * (*this * v) / (v * v);
}

// extrapolate position given bearing (in degrees) and distance
template <typename T>
void Vector2<T>::offset_bearing(T bearing, T distance)
{
  x += cosF(radians(bearing)) * distance;
  y += sinF(radians(bearing)) * distance;
}

// given a position pos_delta and a velocity v1 produce a vector
// perpendicular to v1 maximising distance from p1
template <typename T>
Vector2<T> Vector2<T>::perpendicular(const Vector2<T>& pos_delta, const Vector2<T>& v1)
{
  const Vector2<T> perpendicular1 = Vector2<T>(-v1[1], v1[0]);
  const Vector2<T> perpendicular2 = Vector2<T>(v1[1], -v1[0]);
  const T d1 = perpendicular1 * pos_delta;
  const T d2 = perpendicular2 * pos_delta;
  if (d1 > d2)
  {
    return perpendicular1;
  }
  return perpendicular2;
}

/*
 * Returns the point closest to p on the line segment (v,w).
 *
 * Comments and implementation taken from
 * http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
 */
template <typename T>
Vector2<T> Vector2<T>::closest_point(const Vector2<T>& p, const Vector2<T>& v, const Vector2<T>& w)
{
  // length squared of line segment
  const T l2 = (v - w).length_squared();
  if (l2 < FLT_EPSILON)
  {
    // v == w case
    return v;
  }
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line.
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  // We clamp t from [0,1] to handle points outside the segment vw.
  const T t = ((p - v) * (w - v)) / l2;
  if (t <= 0)
  {
    return v;
  }
  else if (t >= 1)
  {
    return w;
  }
  else
  {
    return v + (w - v) * t;
  }
}

/*
 * Returns the point closest to p on the line segment (0,w).
 *
 * this is a simplification of closest point with a general segment, with v=(0,0)
 */
template <typename T>
Vector2<T> Vector2<T>::closest_point(const Vector2<T>& p, const Vector2<T>& w)
{
  // length squared of line segment
  const T l2 = w.length_squared();
  if (l2 < FLT_EPSILON)
  {
    // v == w case
    return w;
  }
  const T t = (p * w) / l2;
  if (t <= 0)
  {
    return Vector2<T>(0, 0);
  }
  else if (t >= 1)
  {
    return w;
  }
  else
  {
    return w * t;
  }
}

// closest distance between a line segment and a point
// https://stackoverflow.com/questions/2824478/shortest-distance-between-two-line-segments
template <typename T>
T Vector2<T>::closest_distance_between_line_and_point_squared(
  const Vector2<T>& w1,
  const Vector2<T>& w2,
  const Vector2<T>& p)
{
  return closest_distance_between_radial_and_point_squared(w2 - w1, p - w1);
}

// w1 and w2 define a line segment
// p is a point
// returns the closest distance between the line segment and the point
template <typename T>
T Vector2<T>::closest_distance_between_line_and_point(
  const Vector2<T>& w1,
  const Vector2<T>& w2,
  const Vector2<T>& p)
{
  return sqrtF(closest_distance_between_line_and_point_squared(w1, w2, p));
}

// a1->a2 and b2->v2 define two line segments
// returns the square of the closest distance between the two line segments
// see https://stackoverflow.com/questions/2824478/shortest-distance-between-two-line-segments
template <typename T>
T Vector2<T>::closest_distance_between_lines_squared(
  const Vector2<T>& a1,
  const Vector2<T>& a2,
  const Vector2<T>& b1,
  const Vector2<T>& b2)
{
  const T dist1 = Vector2<T>::closest_distance_between_line_and_point_squared(b1, b2, a1);
  const T dist2 = Vector2<T>::closest_distance_between_line_and_point_squared(b1, b2, a2);
  const T dist3 = Vector2<T>::closest_distance_between_line_and_point_squared(a1, a2, b1);
  const T dist4 = Vector2<T>::closest_distance_between_line_and_point_squared(a1, a2, b2);
  const T m1 = MIN(dist1, dist2);
  const T m2 = MIN(dist3, dist4);
  return MIN(m1, m2);
}

// w defines a line segment from the origin
// p is a point
// returns the square of the closest distance between the radial and the point
template <typename T>
T Vector2<T>::closest_distance_between_radial_and_point_squared(
  const Vector2<T>& w,
  const Vector2<T>& p)
{
  const Vector2<T> closest = closest_point(p, w);
  return (closest - p).length_squared();
}

// w defines a line segment from the origin
// p is a point
// returns the closest distance between the radial and the point
template <typename T>
T Vector2<T>::closest_distance_between_radial_and_point(const Vector2<T>& w, const Vector2<T>& p)
{
  return sqrtF(closest_distance_between_radial_and_point_squared(w, p));
}

// rotate vector by angle in radians
template <typename T>
void Vector2<T>::rotate(T angle_rad)
{
  const T cs = cosF(angle_rad);
  const T sn = sinF(angle_rad);
  T rx = x * cs - y * sn;
  T ry = x * sn + y * cs;
  x = rx;
  y = ry;
}

// define for float and double
template class Vector2<float>;
template class Vector2<double>;

// define some ops for int and long
template bool Vector2<long>::operator==(const Vector2<long>& v) const;
template bool Vector2<long>::operator!=(const Vector2<long>& v) const;
template bool Vector2<int>::operator==(const Vector2<int>& v) const;
template bool Vector2<int>::operator!=(const Vector2<int>& v) const;
}  // namespace tobas_mr_arducopter
