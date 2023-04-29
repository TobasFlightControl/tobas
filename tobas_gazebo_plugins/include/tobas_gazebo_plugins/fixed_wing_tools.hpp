#pragma once

#include <vector>
#include <gazebo/gazebo.hh>

#include <dh_std_tools/range.hpp>

namespace gazebo
{
struct VehicleParameters
{
  double wing_surface;                          // 主翼面積 [m^2]
  double wing_span;                             // 翼幅 [m]
  double mean_aerodynamic_chord;                // 平均空力翼弦 [m]
  ignition::math::Vector3d aerodynamic_center;  // フレーム原点に対する空力中心 (NWU) [m]
};

struct AerodynamicsCoefficients
{
  // Lift force
  double c_lift_0;      // [-]
  double c_lift_alpha;  // [/rad]

  // Drag force
  double c_drag_0;      // [-]
  double c_drag_alpha;  // [/rad]

  // Side force
  double c_side_beta;  // [/rad]

  // Roll moment
  double c_roll_beta;  // [/rad]
  double c_roll_p;     // [/rad]
  double c_roll_r;     // [/rad]

  // Pitch moment
  double c_pitch_0;           // [-]
  double c_pitch_alpha;       // [/rad]
  double c_pitch_abs_beta;    // [/rad]
  double c_pitch_alpha_rate;  // [/rad]
  double c_pitch_q;           // [/rad]

  // Yaw moment
  double c_yaw_beta;  // [/rad]
  double c_yaw_p;     // [/rad]
  double c_yaw_r;     // [/rad]
};

/**
 * @brief 舵面．軸が概ねY軸またはZ軸に平行であることを想定．
 */
class ControlSurface
{
public:
  uint32_t index;  // 舵角配列における添字
  dh_std::Range<double> angle_limit;
  double max_angle_rate;

  double c_lift_delta;      // [/rad]
  double c_drag_abs_delta;  // [/rad], 舵角の正負にかかわらず抗力が発生するモデル
  double c_side_delta;      // [/rad]
  double c_roll_delta;      // [/rad]
  double c_pitch_delta;     // [/rad]
  double c_yaw_delta;       // [/rad]

  explicit ControlSurface();

  void setAngle(double cmd_angle, double dt);
  const double& getAngle() const;

private:
  double angle_;
};
}  // namespace gazebo
