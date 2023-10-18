#pragma once

namespace tobas_mr_arducopter
{
struct PidInfo
{
  double target;
  double actual;
  double error;
  double P;
  double I;
  double D;
  double FF;
  double Dmod;
  double slew_rate;
  bool limit;
};
}  // namespace tobas_mr_arducopter
