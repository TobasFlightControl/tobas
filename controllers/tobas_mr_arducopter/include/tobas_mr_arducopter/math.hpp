#pragma once

namespace tobas_mr_arducopter
{
double radians(double deg);
double degrees(double rad);

/* calculate a low pass filter alpha value */
double calcLowPassAlphaDt(double dt, double cutoff_freq);

double wrap_PI(const double radian);
double wrap_2PI(const double radian);
}  // namespace tobas_mr_arducopter
