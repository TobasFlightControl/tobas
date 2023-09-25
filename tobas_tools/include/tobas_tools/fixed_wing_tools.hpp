#pragma once

#include <vector>
#include <Eigen/Core>
#include <kdl/frames.hpp>

#include <dh_std_tools/range.hpp>

namespace tobas
{
struct VehicleParameters
{
  double wing_surface;                // Wing surface [m^2]
  double wing_span;                   // Wing span [m]
  double mac;                         // Mean Aerodynamic Chord [m]
  KDL::Vector ac;                     // Aerodynamic Center wrt the frame origin (NWU) [m]
  dh_std::Range<double> alpha_limit;  // Stall angles [rad]
};

/**
 * @brief Aerodynamics stability derivatives independent of control surfaces.
 * The moment reference point for the wing is at the wing quarter-chord.
 */
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
  double c_roll_p;     // [s/rad]
  double c_roll_r;     // [s/rad]

  // Pitch moment
  double c_pitch_0;           // [-]
  double c_pitch_alpha;       // [/rad]
  double c_pitch_abs_beta;    // [/rad]
  double c_pitch_alpha_rate;  // [s/rad]
  double c_pitch_q;           // [s/rad]

  // Yaw moment
  double c_yaw_beta;  // [/rad]
  double c_yaw_p;     // [s/rad]
  double c_yaw_r;     // [s/rad]
};

/**
 * @brief Control sufrace.
 * The moment reference point for the wing is at the wing quarter-chord.
 * A rotation axis parallel to the Y or Z axis is assumed.
 */
struct ControlSurface
{
  int index;  // 舵角配列における添字
  std::string joint_name;
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

/**
 * @brief 迎角 (alpha) を計算する．
 *
 * @param u,w 風に対する相対的な機体速度 (NED座標系) [m/s]
 * @return double 迎角 [rad]
 */
double angleOfAttack(const double& u, const double& w);

/**
 * @brief 迎角 (alpha) を計算する．
 *
 * @param linvel_B 風に対する相対的な機体速度 (NED座標系) [m/s]
 * @return double 迎角 [rad]
 */
double angleOfAttack(const KDL::Vector& linvel_B);

/**
 * @brief 横滑り角 (beta) を計算する．
 *
 * @param u,v,w 風に対する相対的な機体速度 (NED座標系) [m/s]
 * @return double 横滑り角 [rad]
 */
double angleOfSideSlip(const double& u, const double& v, const double& w);

/**
 * @brief 横滑り角 (beta) を計算する．
 *
 * @param linvel_B 風に対する相対的な機体速度 (NED座標系) [m/s]
 * @return double 横滑り角 [rad]
 */
double angleOfSideSlip(const KDL::Vector& linvel_B);

/**
 * @brief 動圧 (q_bar) を計算する．
 *
 * @param rho 大気密度 [kg/m^3]
 * @param V 風に対する相対的な機体速度の絶対値 [m/s]
 * @return double 動圧 [Pa]
 */
double dynamicPressure(const double& rho, const double& V);
}  // namespace tobas
