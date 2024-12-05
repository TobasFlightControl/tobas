#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_drone_tools/fw_micro_disturbance_eom.hpp"
#include "../include/tobas_drone_tools/utils/fixed_wing_tools.hpp"

#define X_AXIS Vector3d(1, 0, 0)

using namespace std;
using namespace Eigen;

namespace tobas
{
MicroDisturbanceEoM::MicroDisturbanceEoM(const Drone& drone, const kdl::Tree& tree)
  : drone_(drone),
    tree_(tree),
    fk_solver_(tree),
    inertia_solver_(tree),
    x_rotors_(drone, X_POSITIVE),
    trim_(drone, tree)
{
  if (drone.fixed_wing.equipped)
    updateInternalDataStructures();
}

void MicroDisturbanceEoM::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  x_rotors_.updateInternalDataStructures();
  trim_.updateInternalDataStructures();

  u_size_ = x_rotors_.count() + drone_.numControlSurfaces();

  x_0_ = Matrix<double, kStateSize, 1>::Zero();
  u_0_ = VectorXd::Zero(u_size_);
  A_ = Matrix<double, kStateSize, kStateSize>::Zero();
  B_ = MatrixXd::Zero(kStateSize, u_size_);
}

int MicroDisturbanceEoM::update(
  const double& V,
  const double& rho,
  const double& battery_voltage,
  const kdl::JntArray& q)
{
  assert(V > 0.);
  assert(rho > 0.);
  assert(battery_voltage);
  assert(q.rows() == tree_.getNrOfJoints());

  error_code_ = E_NO_ERROR;

  // 制御入力の制約を更新
  setInputLimits(battery_voltage);

  // トリム状態を更新
  trim_.update(V, rho, q);
  if (updateError(trim_) <= E_ERROR)
    return error_code_;

  // エイリアス
  const auto& vehicle = drone_.fixed_wing.vehicle;
  const auto& aero = drone_.fixed_wing.aerodynamics;
  const auto& asd_cog = trim_.stabilityDerivativesCG();

  // 重心と慣性テンソル
  if (inertia_solver_.JntToCart(q) < 0)
  {
    error_msg_ = inertia_solver_.errorMessage();
    return error_code_ = E_ERROR;
  }
  const auto& I_base = inertia_solver_.getInertia();
  const auto P_base_cog = I_base.getCOG();
  const auto I_cog = I_base.getRotationalInertiaCoG();
  // TODO: CoGが許容範囲内にあることとX軸対称性をチェック
  const auto I_x = I_cog.ixx();
  const auto I_y = I_cog.iyy();
  const auto I_z = I_cog.izz();
  const auto I_xz = I_cog.ixz();

  // p.97
  const auto tmp = 1 - math::sqr(I_xz) / (I_x * I_z);
  const auto I_x_tilde = I_x * tmp;
  const auto I_z_tilde = I_z * tmp;

  // 引数に依存する定数
  const auto q_bar = dynamicPressure(rho, V);
  const auto q_S = q_bar * vehicle.wing_surface;
  const auto q_S_b = q_S * vehicle.wing_span;
  const auto q_S_c = q_S * vehicle.mac;
  const auto rho_V_S = rho * V * vehicle.wing_surface;
  const auto rho_V_S_b2 = rho_V_S * math::sqr(vehicle.wing_span);
  const auto rho_V_S_c2 = rho_V_S * math::sqr(vehicle.mac);
  const auto P = I_base.getMass() * V;  // 運動量

  // (2.2-45)
  const auto X_u = -rho_V_S / I_base.getMass() * trim_.c_D();
  const auto X_alpha = -q_S / I_base.getMass() * (aero.c_drag_alpha - trim_.c_L());

  // (3.2-20)
  const auto Y_beta_bar = q_S / P * aero.c_side_beta;

  // (2.2-46)
  const auto Z_u_bar = -rho * vehicle.wing_surface / I_base.getMass() * trim_.c_L();
  const auto Z_alpha_bar = -q_S / P * (aero.c_lift_alpha + 2 * trim_.c_L() * tan(trim_.alpha()));

  // (3.2-21)
  const auto L_beta_dash = q_S_b / I_x_tilde * (aero.c_roll_beta + I_xz / I_z * asd_cog.cYawBeta());
  const auto L_p_dash = rho_V_S_b2 / 4 / I_x_tilde * (aero.c_roll_p + I_xz / I_z * aero.c_yaw_p);
  const auto L_r_dash = rho_V_S_b2 / 4 / I_x_tilde * (aero.c_roll_r + I_xz / I_z * aero.c_yaw_r);

  // (2.2-47)
  const auto M_u = 0.;
  const auto M_alpha = q_S_c / I_y * asd_cog.cPitchAlpha();
  const auto M_q = rho_V_S_c2 / 4 / I_y * aero.c_pitch_q;
  const auto M_alpha_rate = rho_V_S_c2 / 4 / I_y * aero.c_pitch_alpha_rate;

  // (2.2-39)
  const auto M_u_dash = M_u + M_alpha_rate * Z_u_bar;
  const auto M_alpha_dash = M_alpha + M_alpha_rate * Z_alpha_bar;
  const auto M_q_dash = M_q + M_alpha_rate;
  const auto M_theta_dash = -tobas_std::kGravity * sin(trim_.theta()) / V * M_alpha_rate;

  // (3.2-22)
  const auto N_beta_dash = q_S_b / I_z_tilde * (asd_cog.cYawBeta() + I_xz / I_x * aero.c_roll_beta);
  const auto N_p_dash = rho_V_S_b2 / 4 / I_z_tilde * (aero.c_yaw_p + I_xz / I_x * aero.c_roll_p);
  const auto N_r_dash = rho_V_S_b2 / 4 / I_z_tilde * (aero.c_yaw_r + I_xz / I_x * aero.c_roll_r);

  // Aを更新
  A_(kStateIdx_u, kStateIdx_u) = X_u;
  A_(kStateIdx_u, kStateIdx_alpha) = X_alpha;
  A_(kStateIdx_u, kStateIdx_theta) = -tobas_std::kGravity * cos(trim_.theta());

  A_(kStateIdx_alpha, kStateIdx_u) = Z_u_bar;
  A_(kStateIdx_alpha, kStateIdx_alpha) = Z_alpha_bar;
  A_(kStateIdx_alpha, kStateIdx_theta) = -tobas_std::kGravity * sin(trim_.theta()) / V;
  A_(kStateIdx_alpha, kStateIdx_q) = 1;

  A_(kStateIdx_beta, kStateIdx_beta) = Y_beta_bar;
  A_(kStateIdx_beta, kStateIdx_phi) = tobas_std::kGravity * cos(trim_.theta()) / V;
  A_(kStateIdx_beta, kStateIdx_p) = trim_.alpha();
  A_(kStateIdx_beta, kStateIdx_r) = -1;

  A_(kStateIdx_phi, kStateIdx_p) = 1;
  A_(kStateIdx_phi, kStateIdx_r) = tan(trim_.theta());

  A_(kStateIdx_theta, kStateIdx_q) = 1;

  A_(kStateIdx_p, kStateIdx_beta) = L_beta_dash;
  A_(kStateIdx_p, kStateIdx_p) = L_p_dash;
  A_(kStateIdx_p, kStateIdx_r) = L_r_dash;

  A_(kStateIdx_q, kStateIdx_u) = M_u_dash;
  A_(kStateIdx_q, kStateIdx_alpha) = M_alpha_dash;
  A_(kStateIdx_q, kStateIdx_theta) = M_theta_dash;
  A_(kStateIdx_q, kStateIdx_q) = M_q_dash;

  A_(kStateIdx_r, kStateIdx_beta) = N_beta_dash;
  A_(kStateIdx_r, kStateIdx_p) = N_p_dash;
  A_(kStateIdx_r, kStateIdx_r) = N_r_dash;

  // Bを更新
  // thrust -> u
  for (size_t i = 0; i < x_rotors_.count(); ++i)
    B_(kStateIdx_u, i) = 1 / I_base.getMass();

  // thrust -> p,q,r
  const auto I_cog_inv = I_cog.data.inverse();
  for (size_t i = 0; i < x_rotors_.count(); ++i)
  {
    const auto& rotor = x_rotors_.rotor(i);
    if (fk_solver_.JntToCart(q, rotor.link_name) < 0)
    {
      error_msg_ = fk_solver_.errorMessage();
      return error_code_ = E_ERROR;
    }
    const auto P_cog_rotor = fk_solver_.getFrame().p - P_base_cog;
    const auto d = rotor.sign();
    const auto& c = rotor.moment_constant;
    Vector3d v = I_cog_inv * (P_cog_rotor.data.cross(X_AXIS) - (d * c) * X_AXIS);  // NWU
    eigen::vectorNwuToNed(v);                                                      // NWU -> NED
    B_.block(kStateIdx_p, i, 3, 1) = v;
  }

  // deflection
  size_t cs_idx = 0;
  for (const auto& [channel, cs] : drone_.fixed_wing.control_surfaces)
  {
    const auto pitch_delta = asd_cog.cPitchDelta(channel);
    const auto yaw_delta = asd_cog.cYawDelta(channel);

    const auto Y_delta_bar = q_S / P * cs.c_side_delta;                                        // (3.2-20)
    const auto Z_delta_bar = -q_S / P * cs.c_lift_delta;                                       // (2.2-37)
    const auto L_delta_dash = q_S_b / I_x_tilde * (cs.c_roll_delta + I_xz / I_z * yaw_delta);  // (3.2-21)
    const auto M_delta = q_S_c / I_y * pitch_delta;                                            // (2.2-38)
    const auto M_delta_dash = M_delta + M_alpha_rate * Z_delta_bar;                            // (2.2-39)
    const auto N_delta_dash = q_S_b / I_z_tilde * (yaw_delta + I_xz / I_x * cs.c_roll_delta);  // (3.2-22)

    const auto col = x_rotors_.count() + cs_idx;
    B_(kStateIdx_alpha, col) = Z_delta_bar;
    B_(kStateIdx_beta, col) = Y_delta_bar;
    B_(kStateIdx_p, col) = L_delta_dash;
    B_(kStateIdx_q, col) = M_delta_dash;
    B_(kStateIdx_r, col) = N_delta_dash;

    ++cs_idx;
  }

  // トリム時の状態を更新
  x_0_(kStateIdx_u) = trim_.u();
  x_0_(kStateIdx_alpha) = trim_.alpha();
  x_0_(kStateIdx_beta) = 0.;
  x_0_(kStateIdx_phi) = 0.;
  x_0_(kStateIdx_theta) = trim_.theta();
  x_0_(kStateIdx_p) = 0.;
  x_0_(kStateIdx_q) = 0.;
  x_0_(kStateIdx_r) = 0.;

  // トリム時の制御入力を更新
  const auto thrust_sum = q_S * trim_.c_T();  // (2.2-2b)
  for (size_t i = 0; i < x_rotors_.count(); ++i)
  {
    auto thrust = thrust_sum / x_rotors_.count();  // TODO: 横の釣り合いも考慮して分配
    const auto max_thrust = x_rotors_.rotor(i).thrustFromVoltage(battery_voltage);
    if (thrust > max_thrust)
    {
      if (error_code_ > E_WARN)
      {
        error_code_ = E_WARN;
        error_msg_ = "Thrust force is over the maximum limit: " + to_string(thrust) + " > " + to_string(max_thrust);
      }
      thrust = max_thrust;
    }
    u_0_(i) = thrust;
  }
  u_0_(x_rotors_.count() + trim_.elevatorChannel()) = trim_.elevator();

  return error_code_;
}

void MicroDisturbanceEoM::setInputLimits(const double& battery_voltage)
{
  min_u_.conservativeResize(u_size_);
  max_u_.conservativeResize(u_size_);

  for (size_t i = 0; i < x_rotors_.count(); ++i)
  {
    const auto& rotor = x_rotors_.rotor(i);
    min_u_(i) = rotor.minThrust(battery_voltage);
    max_u_(i) = rotor.maxThrust(battery_voltage);
  }

  size_t cs_idx = 0;
  for (const auto& [_, cs] : drone_.fixed_wing.control_surfaces)
  {
    min_u_(x_rotors_.count() + cs_idx) = cs.angle_limit.lower;
    max_u_(x_rotors_.count() + cs_idx) = cs.angle_limit.upper;
    ++cs_idx;
  }
}
}  // namespace tobas
