#pragma once

#include <cmath>

namespace tobas::ctrl
{
class PIController
{
public:
  PIController();
  PIController(double _p, double _i);
  void setGains(double _p, double _i);
  void setMaxMin(double _max, double _min);
  void reset();
  void update(double _u, double _dt);

  double getValue() const;

private:
  double p_;
  double i_;

  double integral_ = 0.0;
  double saturation_ = 0.0;

  double input_ = 0.0;

  double max_ = INFINITY;
  double min_ = -INFINITY;
};
}  // namespace tobas::ctrl
