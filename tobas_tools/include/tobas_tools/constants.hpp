#pragma once

#include <cinttypes>

namespace tobas
{
constexpr double kGravity = 9.80665;        // 重力加速度 [m/s^2]
constexpr double kMinAirSpeedThresh = 0.1;  // 空力計算を行う最小の風速 [m/s]
constexpr uint32_t kMinPinId = 1;
constexpr uint32_t kMaxPinId = 14;

// モータが停止して静止摩擦が発生することを防ぐために，最小スロットル率を設定．
// cf. https://ardupilot.org/copter/docs/set-motor-range.html
// TODO: Ardupilotを参考にGUIで設定できるようにする
static constexpr double kMotorSpinArm = 0.1;

static constexpr char kLandingAction[] = "landing_action";
static constexpr char kTakeoffAction[] = "takeoff_action";
}  // namespace tobas
