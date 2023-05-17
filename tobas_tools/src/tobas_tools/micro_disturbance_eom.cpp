#include <eigen_conversions/eigen_kdl.h>

#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_kdl/conversion/kdl_eigen.hpp>

#include "../../include/tobas_tools/micro_disturbance_eom.hpp"
#include "../../include/tobas_tools/constants.hpp"

#define X_AXIS Vector3d(1, 0, 0)

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas
{
MicroDisturbanceEoM::MicroDisturbanceEoM(const Drone& drone, const uint32_t& elev_cs_idx)
  : drone_(drone),
    fk_solver_(drone.tree()),
    inertia_solver_(drone.tree()),
    x_rotors_(drone, Axis::X_POSITIVE),
    trim_(drone, elev_cs_idx)
{
  if (drone.isLoaded())
  {
    updateInternalDataStructures();
  }
}

void MicroDisturbanceEoM::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  x_rotors_.updateInternalDataStructures();
  trim_.updateInternalDataStructures();

  mass_ = inertia_solver_.JntToMass();
  num_hprop_ = x_rotors_.count();
  num_cs_ = drone_.fixedWingConfig().control_surfaces.size();
  u_size_ = num_hprop_ + num_cs_;
  setInputLimits();

  x_0_ = Matrix<double, kStateSize, 1>::Zero();
  u_0_ = VectorXd::Zero(u_size_);
  A_ = Matrix<double, kStateSize, kStateSize>::Zero();
  B_ = MatrixXd::Zero(kStateSize, u_size_);
}

void MicroDisturbanceEoM::update(double V, double h, const JntArray& q)
{
  assert(V > 0.);
  assert(h > 0.);
  assert(q.rows() == drone_.tree().getNrOfJoints());

  // エイリアス
  const auto& fixed_wing = drone_.fixedWingConfig();
  const auto& vehicle = fixed_wing.vehicle;
  const auto& aero = fixed_wing.aerodynamics;
  const auto& control_surfaces = fixed_wing.control_surfaces;

  // 重心と慣性テンソル
  inertia_solver_.JntToCart(q, P_base_cog_, I_kdl_);
  // TODO: CoGが許容範囲内にあることとX軸対称性をチェック
  const auto I_x = I_kdl_.data[0];
  const auto I_y = I_kdl_.data[4];
  const auto I_z = I_kdl_.data[8];
  const auto I_xz = I_kdl_.data[2];
  const auto tmp = 1 - sqr(I_xz) / (I_x * I_z);
  const auto I_x_tilde = I_x * tmp;
  const auto I_z_tilde = I_z * tmp;

  // トリム状態を更新
  trim_.update(V, h, q);
  const auto& asd_cog = trim_.stabilityDerivativesCG();

  // 引数に依存する定数
  const auto rho = dh_std::altitudeToDensity(h);
  const auto W = mass_ * kGravity;
  const auto q_bar = dynamicPressure(rho, V);
  const auto q_S = q_bar * vehicle.wing_surface;
  const auto q_S_b = q_S * vehicle.wing_span;
  const auto q_S_c = q_S * vehicle.mac;
  const auto rho_V_S = rho * V * vehicle.wing_surface;
  const auto rho_V_S_b2 = rho_V_S * sqr(vehicle.wing_span);
  const auto P = mass_ * V;  // 運動量

  // (2.2-45)
  const auto X_u = -rho_V_S / mass_ * trim_.c_D();
  const auto X_alpha = -q_S / mass_ * (aero.c_drag_alpha - trim_.c_L());

  // (3.2-20)
  const auto Y_beta_bar = q_S / P * aero.c_side_beta;

  // (2.2-46)
  const auto Z_u_bar = -rho * vehicle.wing_surface / mass_ * trim_.c_L();
  const auto Z_alpha_bar = -q_S / P * (aero.c_lift_alpha + 2 * trim_.c_L() * tan(trim_.alpha()));

  // (3.2-21)
  const auto L_beta_dash = q_S_b / I_x_tilde * (aero.c_roll_beta + I_xz / I_z * asd_cog.cYawBeta());
  const auto L_p_dash = rho_V_S_b2 / 4 / I_x_tilde * (aero.c_roll_p + I_xz / I_z * aero.c_yaw_p);
  const auto L_r_dash = rho_V_S_b2 / 4 / I_x_tilde * (aero.c_roll_r + I_xz / I_z * aero.c_yaw_r);

  // (2.2-47)
  const auto M_u = 0.;
  const auto M_alpha = q_S_c / I_y * asd_cog.cPitchAlpha();
  const auto M_q = rho_V_S * sqr(vehicle.mac) / 4 / I_y * aero.c_pitch_q;
  const auto M_alpha_rate = rho_V_S * sqr(vehicle.mac) / 4 / I_y * aero.c_pitch_alpha_rate;

  // (2.2-39)
  const auto M_u_dash = M_u + M_alpha_rate * Z_u_bar;
  const auto M_alpha_dash = M_alpha + M_alpha_rate * Z_alpha_bar;
  const auto M_q_dash = M_q + M_alpha_rate;
  const auto M_theta_dash = -kGravity * sin(trim_.theta()) / V * M_alpha_rate;

  // (3.2-22)
  const auto N_beta_dash = q_S_b / I_z_tilde * (asd_cog.cYawBeta() + I_xz / I_x * aero.c_roll_beta);
  const auto N_p_dash = rho_V_S_b2 / 4 / I_z_tilde * (aero.c_yaw_p + I_xz / I_x * aero.c_roll_p);
  const auto N_r_dash = rho_V_S_b2 / 4 / I_z_tilde * (aero.c_yaw_r + I_xz / I_x * aero.c_roll_r);

  // Aを更新
  A_(kStateIdx_u, kStateIdx_u) = X_u;
  A_(kStateIdx_u, kStateIdx_alpha) = X_alpha;
  A_(kStateIdx_u, kStateIdx_theta) = -kGravity * cos(trim_.theta());

  A_(kStateIdx_alpha, kStateIdx_u) = Z_u_bar;
  A_(kStateIdx_alpha, kStateIdx_alpha) = Z_alpha_bar;
  A_(kStateIdx_alpha, kStateIdx_theta) = -kGravity * sin(trim_.theta()) / V;
  A_(kStateIdx_alpha, kStateIdx_q) = 1;

  A_(kStateIdx_beta, kStateIdx_beta) = Y_beta_bar;
  A_(kStateIdx_beta, kStateIdx_phi) = kGravity * cos(trim_.theta()) / V;
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
  for (int i = 0; i < num_hprop_; ++i)
  {
    B_(kStateIdx_u, i) = 1 / mass_;
  }

  // thrust -> p,q,r
  tf::rotInertiaKDLToEigen(I_kdl_, I_eigen_);
  const auto I_inv = I_eigen_.inverse();
  for (int i = 0; i < num_hprop_; ++i)
  {
    fk_solver_.JntToCart(q, x_rotors_.linkName(i), T_base_rotor_);
    const auto P_cog_rotor_kdl = T_base_rotor_.p - P_base_cog_;
    tf::vectorKDLToEigen(P_cog_rotor_kdl, P_cog_rotor_eigen_);
    const auto& d = x_rotors_.direction(i);
    const auto& c = x_rotors_.momentConstant(i);
    B_.block(kStateIdx_p, i, 3, 1) = I_inv * (P_cog_rotor_eigen_.cross(X_AXIS) - (d * c) * X_AXIS);
  }

  // deflection
  for (int cs_idx = 0; cs_idx < num_cs_; ++cs_idx)
  {
    const auto& cs = control_surfaces[cs_idx];

    const auto Y_delta_bar = q_S / P * cs.c_side_delta;                                // (3.2-20)
    const auto Z_delta_bar = -q_S / P * cs.c_lift_delta;                               // (2.2-37)
    const auto L_delta_dash =
      q_S_b / I_x_tilde * (cs.c_roll_delta + I_xz / I_z * asd_cog.cYawDelta(cs_idx));  // (3.2-21)
    const auto M_delta = q_S_c / I_y * asd_cog.cPitchDelta(cs_idx);                    // (2.2-38)
    const auto M_delta_dash = M_delta + M_alpha_rate * Z_delta_bar;                    // (2.2-39)
    const auto N_delta_dash =
      q_S_b / I_z_tilde * (asd_cog.cYawDelta(cs_idx) + I_xz / I_x * cs.c_roll_delta);  // (3.2-22)

    const auto col = num_hprop_ + cs_idx;
    B_(kStateIdx_alpha, col) = Z_delta_bar;
    B_(kStateIdx_beta, col) = Y_delta_bar;
    B_(kStateIdx_p, col) = L_delta_dash;
    B_(kStateIdx_q, col) = M_delta_dash;
    B_(kStateIdx_r, col) = N_delta_dash;
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
  // TODO: 横の釣り合いも考慮して分配
  const auto thrust_sum = q_S * trim_.c_T();  // (2.2-2b)
  const auto thrust_avg = thrust_sum / num_hprop_;
  u_0_.block(0, 0, num_hprop_, 0) = VectorXd::Constant(num_hprop_, thrust_avg);
  u_0_(num_hprop_) = trim_.elevator();
}

const TrimConditions& MicroDisturbanceEoM::trimCondition() const
{
  return trim_;
}

const StabilityDerivativesCG& MicroDisturbanceEoM::stabilityDerivativesCG() const
{
  return trim_.stabilityDerivativesCG();
}

const Matrix<double, MicroDisturbanceEoM::kStateSize, 1>& MicroDisturbanceEoM::trimState() const
{
  return x_0_;
}

const VectorXd& MicroDisturbanceEoM::trimInput() const
{
  return u_0_;
}

const VectorXd& MicroDisturbanceEoM::minInput() const
{
  return min_u_;
}

const VectorXd& MicroDisturbanceEoM::maxInput() const
{
  return max_u_;
}

VectorXd MicroDisturbanceEoM::minDeltaInput() const
{
  return min_u_ - u_0_;
}

VectorXd MicroDisturbanceEoM::maxDeltaInput() const
{
  return max_u_ - u_0_;
}

const uint32_t& MicroDisturbanceEoM::elevatorIndex() const
{
  return trim_.elevatorIndex();
}

const uint32_t& MicroDisturbanceEoM::inputSize() const
{
  return u_size_;
}

const Matrix<double, MicroDisturbanceEoM::kStateSize, MicroDisturbanceEoM::kStateSize>&
MicroDisturbanceEoM::A()
{
  return A_;
}

const Matrix<double, MicroDisturbanceEoM::kStateSize, Dynamic>& MicroDisturbanceEoM::B()
{
  return B_;
}

const double& MicroDisturbanceEoM::u_u() const
{
  return A_(kStateIdx_u, kStateIdx_u);
}

const double& MicroDisturbanceEoM::u_alpha() const
{
  return A_(kStateIdx_u, kStateIdx_alpha);
}

const double& MicroDisturbanceEoM::u_q() const
{
  return A_(kStateIdx_u, kStateIdx_q);
}

const double& MicroDisturbanceEoM::u_theta() const
{
  return A_(kStateIdx_u, kStateIdx_theta);
}

const double& MicroDisturbanceEoM::alpha_u() const
{
  return A_(kStateIdx_alpha, kStateIdx_u);
}

const double& MicroDisturbanceEoM::alpha_alpha() const
{
  return A_(kStateIdx_alpha, kStateIdx_alpha);
}

const double& MicroDisturbanceEoM::alpha_q() const
{
  return A_(kStateIdx_alpha, kStateIdx_q);
}

const double& MicroDisturbanceEoM::alpha_theta() const
{
  return A_(kStateIdx_alpha, kStateIdx_theta);
}

const double& MicroDisturbanceEoM::beta_beta() const
{
  return A_(kStateIdx_beta, kStateIdx_beta);
}

const double& MicroDisturbanceEoM::beta_p() const
{
  return A_(kStateIdx_beta, kStateIdx_p);
}

const double& MicroDisturbanceEoM::beta_r() const
{
  return A_(kStateIdx_beta, kStateIdx_r);
}

const double& MicroDisturbanceEoM::beta_phi() const
{
  return A_(kStateIdx_beta, kStateIdx_phi);
}

const double& MicroDisturbanceEoM::phi_beta() const
{
  return A_(kStateIdx_phi, kStateIdx_beta);
}

const double& MicroDisturbanceEoM::phi_p() const
{
  return A_(kStateIdx_phi, kStateIdx_p);
}

const double& MicroDisturbanceEoM::phi_r() const
{
  return A_(kStateIdx_phi, kStateIdx_r);
}

const double& MicroDisturbanceEoM::phi_phi() const
{
  return A_(kStateIdx_phi, kStateIdx_phi);
}

const double& MicroDisturbanceEoM::theta_u() const
{
  return A_(kStateIdx_theta, kStateIdx_u);
}

const double& MicroDisturbanceEoM::theta_alpha() const
{
  return A_(kStateIdx_theta, kStateIdx_alpha);
}

const double& MicroDisturbanceEoM::theta_q() const
{
  return A_(kStateIdx_theta, kStateIdx_q);
}

const double& MicroDisturbanceEoM::theta_theta() const
{
  return A_(kStateIdx_theta, kStateIdx_theta);
}

const double& MicroDisturbanceEoM::p_beta() const
{
  return A_(kStateIdx_p, kStateIdx_beta);
}

const double& MicroDisturbanceEoM::p_p() const
{
  return A_(kStateIdx_p, kStateIdx_p);
}

const double& MicroDisturbanceEoM::p_r() const
{
  return A_(kStateIdx_p, kStateIdx_r);
}

const double& MicroDisturbanceEoM::p_phi() const
{
  return A_(kStateIdx_p, kStateIdx_phi);
}

const double& MicroDisturbanceEoM::q_u() const
{
  return A_(kStateIdx_q, kStateIdx_u);
}

const double& MicroDisturbanceEoM::q_alpha() const
{
  return A_(kStateIdx_q, kStateIdx_alpha);
}

const double& MicroDisturbanceEoM::q_q() const
{
  return A_(kStateIdx_q, kStateIdx_q);
}

const double& MicroDisturbanceEoM::q_theta() const
{
  return A_(kStateIdx_q, kStateIdx_theta);
}

const double& MicroDisturbanceEoM::r_beta() const
{
  return A_(kStateIdx_r, kStateIdx_beta);
}

const double& MicroDisturbanceEoM::r_p() const
{
  return A_(kStateIdx_r, kStateIdx_p);
}

const double& MicroDisturbanceEoM::r_r() const
{
  return A_(kStateIdx_r, kStateIdx_r);
}

const double& MicroDisturbanceEoM::r_phi() const
{
  return A_(kStateIdx_r, kStateIdx_phi);
}

double MicroDisturbanceEoM::u_thrust() const
{
  return 1 / mass_;
}

const double& MicroDisturbanceEoM::alpha_delta(uint32_t cs_idx) const
{
  return B_(kStateIdx_alpha, num_hprop_ + cs_idx);
}

const double& MicroDisturbanceEoM::beta_delta(uint32_t cs_idx) const
{
  return B_(kStateIdx_beta, num_hprop_ + cs_idx);
}

const double& MicroDisturbanceEoM::p_delta(uint32_t cs_idx) const
{
  return B_(kStateIdx_p, num_hprop_ + cs_idx);
}

const double& MicroDisturbanceEoM::q_delta(uint32_t cs_idx) const
{
  return B_(kStateIdx_q, num_hprop_ + cs_idx);
}

const double& MicroDisturbanceEoM::r_delta(uint32_t cs_idx) const
{
  return B_(kStateIdx_r, num_hprop_ + cs_idx);
}

void MicroDisturbanceEoM::setInputLimits()
{
  min_u_.resize(u_size_);
  max_u_.resize(u_size_);

  for (int i = 0; i < num_hprop_; ++i)
  {
    min_u_(i) = 0.;
    max_u_(i) = x_rotors_.maxThrust(i);
  }

  for (int i = 0; i < num_cs_; ++i)
  {
    const auto& cs = drone_.fixedWingConfig().control_surfaces[i];
    min_u_(num_hprop_ + i) = cs.angle_limit.lower;
    max_u_(num_hprop_ + i) = cs.angle_limit.upper;
  }
}
}  // namespace tobas
