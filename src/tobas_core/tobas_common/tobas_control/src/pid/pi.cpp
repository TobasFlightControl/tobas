#include "tobas_control/pid/pi.hpp"

#include <algorithm>

namespace tobas::ctrl
{
PIController::PIController() : p_(0.0), i_(0.0), integral_(0.0)
{
}

PIController::PIController(double _p, double _i) : p_(_p), i_(_i), integral_(0.0)
{
}

void PIController::setGains(double _p, double _i)
{
  p_ = _p;
  i_ = _i;
}

void PIController::setMaxMin(double _max, double _min)
{
  max_ = _max;
  min_ = _min;
}

void PIController::reset()
{
  integral_ = 0.0;
}

void PIController::update(double _u, double _dt)
{
  integral_ += (_u - saturation_) * _dt;
  // 出力計算
  const auto input_candidate = p_ * _u + i_ * integral_;
  input_ = std::clamp(input_candidate, min_, max_);

  saturation_ = input_candidate - input_;
}

double PIController::getValue() const
{
  return input_;
}
}  // namespace tobas::ctrl
