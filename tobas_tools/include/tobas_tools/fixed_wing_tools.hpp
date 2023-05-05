#pragma once

#include <vector>
#include <Eigen/Core>

#include <dh_std_tools/range.hpp>

struct VehicleParameters
{
  double wing_surface;                 // 主翼面積 [m^2]
  double wing_span;                    // 翼幅 [m]
  double mean_aerodynamic_chord;       // 平均空力翼弦 [m]
  Eigen::Vector3d aerodynamic_center;  // フレーム原点に対する空力中心 (NWU) [m]
  dh_std::Range<double> alpha_limit;   // 失速角 [rad]
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
struct ControlSurface
{
  int index;  // 舵角配列における添字
  dh_std::Range<double> angle_limit;
  double max_angle_rate;

  double c_lift_delta;      // [/rad]
  double c_drag_abs_delta;  // [/rad], 舵角の正負にかかわらず抗力が発生するモデル
  double c_side_delta;      // [/rad]
  double c_roll_delta;      // [/rad]
  double c_pitch_delta;     // [/rad]
  double c_yaw_delta;       // [/rad]
};

using ControlSurfaces = std::vector<ControlSurface>;

struct FixedWingConfig
{
  VehicleParameters vehicle;
  AerodynamicsCoefficients aerodynamics;
  ControlSurfaces control_surfaces;
};

/* Get fixed wing vehicle parameters from ROS parameter server. */
void getVehicleParameters(VehicleParameters& des);

/* Get aerodynamics coefficients from ROS parameter server. */
void getAerodynamicsCoefficients(AerodynamicsCoefficients& des);

/* Get control surface configurations from ROS parameter server. */
void getControlSurfaces(ControlSurfaces& des);

/* Get fixed wing configurations from ROS parameter server. */
void getFixedWingConfig(FixedWingConfig& des);
