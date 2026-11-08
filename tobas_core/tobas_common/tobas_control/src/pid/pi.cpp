#include "tobas_control/pid/pi.hpp"

#include <algorithm>

namespace tobas
{
namespace ctrl
{
PIController::PIController() : p_(0.0), i_(0.0), integral_(0.0)
{
}

PIController::PIController(double p, double i)
    : p_(p), i_(i), integral_(0.0)
{
}

void PIController::setGains(double p, double i)
{
    p_ = p;
    i_ = i;
}

void PIController::setMaxMin(double max, double min)
{
    max_ = max;
    min_ = min;
}

void PIController::reset()
{
    integral_ = 0.0;
}

void PIController::update(double u, double dt)
{
    integral_ += (u - saturation_) * dt;
    // 出力計算
    const auto input_candidate = p_ * u + i_ * integral_;
    input_ = std::clamp(
        input_candidate,
        min_,
        max_);

    saturation_ = input_candidate - input_;
}

double PIController::getValue() const
{
    return input_;
}
} // namespace ctrl
} // namespace tobas
