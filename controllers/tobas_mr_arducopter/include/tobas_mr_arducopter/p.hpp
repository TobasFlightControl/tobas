#pragma once

#include <cmath>
#include <stdlib.h>

namespace tobas_mr_arducopter
{
// Object managing one P controller
class P
{
public:
  explicit P(const double& initial_p = 0.) : default_kp(initial_p)
  {
  }

  inline double get_p(double error) const;

  // Overload the function call operator to permit relatively easy initialisation
  inline void operator()(const double p);

  // accessors
  inline double& kP();
  inline const double& kP() const;
  inline void kP(const double v);

private:
  double kp_;

  const double default_kp;
};

inline double P::get_p(double error) const
{
  return error * kp_;
}

inline void P::operator()(const double p)
{
  kp_ = p;
}

inline double& P::kP()
{
  return kp_;
}

inline const double& P::kP() const
{
  return kp_;
}

inline void P::kP(const double v)
{
  kp_ = v;
}
}  // namespace tobas_mr_arducopter
