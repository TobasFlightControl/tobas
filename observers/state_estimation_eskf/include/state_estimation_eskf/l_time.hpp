#pragma once

#include <math.h>
#include <stdint.h>

/**
 * @brief An approximation of the ros time class.
 * Though with no templating, reduced error checking and no fancy dependencies.
 */
class lTime
{
public:
  explicit lTime();
  explicit lTime(double t);
  explicit lTime(int32_t _sec, int32_t _nsec);

  void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec);
  void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec);

  double toSec() const;
  int64_t toNSec() const;
  lTime& fromSec(double d);
  lTime& fromNSec(int64_t t);

  bool isZero() const;

  lTime operator+(const lTime& rhs) const;
  lTime operator-(const lTime& rhs) const;
  lTime operator*(double scale) const;
  lTime operator-() const;
  lTime& operator+=(const lTime& rhs);
  lTime& operator-=(const lTime& rhs);
  lTime& operator*=(double scale);

  bool operator<(const lTime& rhs) const;
  bool operator>(const lTime& rhs) const;
  bool operator<=(const lTime& rhs) const;
  bool operator>=(const lTime& rhs) const;
  bool operator==(const lTime& rhs) const;

private:
  int32_t sec, nsec;
  int error;
};
