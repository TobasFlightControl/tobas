#pragma once

#include <cmath>

namespace tobas
{
namespace ctrl
{
class PIController
{
public:
  PIController();
  PIController(double p, double i);
  void setGains(double p, double i);
  void setMaxMin(double max, double min);
  void reset();
  void update(double u, double dt);

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
}  // namespace ctrl
}  // namespace tobas
