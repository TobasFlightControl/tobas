#pragma once

namespace tobas
{
constexpr double kGravity = 9.80665;        // 重力加速度 [m/s^2]
constexpr double kMinAirSpeedThresh = 0.1;  // 空力計算を行う最小の風速 [m/s]
constexpr unsigned int kMinPinId = 1;
constexpr unsigned int kMaxPinId = 14;
}  // namespace tobas
