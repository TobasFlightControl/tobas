#pragma once

namespace tobas_keyboard_teleop
{
static constexpr char kKeyCode_W = 'w';
static constexpr char kKeyCode_S = 's';
static constexpr char kKeyCode_A = 'a';
static constexpr char kKeyCode_D = 'd';
static constexpr char kKeyCode_Up = 0x41;
static constexpr char kKeyCode_Down = 0x42;
static constexpr char kKeyCode_Right = 0x43;
static constexpr char kKeyCode_Left = 0x44;

static constexpr int kFileDescriptor = 0;                   // 標準入力
static constexpr double kUpdateRate = 200.;                 // [Hz]
static constexpr double kInfoPeriod = 1.;                   // [s]
static constexpr double kInstructionTimerPeriod = 10.;      // [s]
static constexpr double kWaitForExternalActionServer = 3.;  // [s]
}  // namespace tobas_keyboard_teleop
