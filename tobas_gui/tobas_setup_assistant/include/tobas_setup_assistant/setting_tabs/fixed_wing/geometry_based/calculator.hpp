#pragma once

#include <QPushButton>

#include <tobas_kdl/tree.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_kdl/twist.hpp>
#include <tobas_kdl/wrench.hpp>

#include <tobas_setup_assistant/setting_tabs/fixed_wing/geometry_based/control_surfaces.hpp>
#include <tobas_setup_assistant/setting_tabs/fixed_wing/geometry_based/wing.hpp>
#include <tobas_setup_assistant/setting_tabs/fixed_wing/geometry_based/wings.hpp>
#include <tobas_setup_assistant/setting_tabs/fixed_wing/manual/manual.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace gb
{
class Calculator : public QPushButton
{
  Q_OBJECT

  using self = Calculator;
  using super = QPushButton;

  static constexpr int kButtonWidth = 125;
  static constexpr int kButtonHeight = 50;
  static constexpr double kDelta = 1.0e-2 * M_PI / 180.0;
  static constexpr double kAffordableRate = 0.1;

public:
  explicit Calculator(
    const kdl::Tree& tree,
    mn::ManualWidget* manual,
    WingsWidget* wings,
    ControlSurfacesWidget* control_surfaces);
  void updateInternalDataStructures();

private:
  const kdl::Tree& tree_;
  kdl::TreeInertiaSolver inertia_solver_;

  const mn::ManualWidget* manual_;
  const WingsWidget* wings_;
  const ControlSurfacesWidget* control_surfaces_;

  double mass_;
  kdl::Vector B_Pos_B2R_;  // base_link座標系でみたときのbase_linkからref座標系までの距離. ref座標系はCoGに取る.

  // C_L = C_L0 + C_Lalpha * alpha
  double c_lift_0_;
  double c_lift_alpha_;
  // C_D = C_D0 + C_Dalpha * alpha + C_Dalpha2 * alpha^2
  double c_drag_0_;
  double c_drag_alpha_;
  // C_Y = C_side_beta * beta + C_side_p * p_hat + C_side_r * r_hat. p_hat = b / 2V * p
  double c_side_beta_;
  double c_side_p_;  // not implemented in coeff widget
  double c_side_r_;  // not implemented in coeff widget
  // C_l = C_lbeta * beta + C_lp * p_hat + C_lr * r_hat
  double c_roll_beta_;
  double c_roll_p_;
  double c_roll_r_;
  // C_m = C_m0 + C_malpha * alpha + C_mq * q_hat + C_mdalpha * dalpha
  double c_pitch_0_;
  double c_pitch_alpha_;
  double c_pitch_abs_beta_ = 0.0;
  double c_pitch_alpha_rate_ = 0.0;
  double c_pitch_q_;
  // C_n = C_mbeta * beta + C_mp * p_hat + C_mr * r_hat
  double c_yaw_beta_;
  double c_yaw_p_;
  double c_yaw_r_;
  // control surface coefficients
  struct CsCoefs
  {
    double lift;
    double drag;
    double side;
    double roll;
    double pitch;
    double yaw;
  };
  std::vector<CsCoefs> control_surface_coefs_;

  void onClicked();
  static void XZ2DL(const kdl::Vector& force, const double& alpha, double& drag, double& lift);
  void calcWingTranslations(const WingWidget* wing, kdl::Frame& W_T_R, kdl::Frame& R_T_W) const;
  void calcMacTranslations(const WingWidget* wing, kdl::Frame& M_T_R, kdl::Frame& R_T_M) const;
  // 機体モーメント基準点のlocal座標系で見たとき, 機体が発生する空力を計算する
  kdl::Wrench calcMachineAeroDynamicForce(const kdl::Twist& twist_ref) const;
  // 指定されたwingのmac座標系で見たとき, そのwingが発生する空力を計算する
  kdl::Wrench calcWingAeroDynamicForce(const WingWidget* wing, const kdl::Twist& twist_mac) const;
  void calcCruiseCoeff(const double& cruise_speed, const double& cruise_alpha);
  void calcAoACoeff(const double& cruise_speed, const double& cruise_alpha);
  void calcAoSCoeff(const double& cruise_speed, const double& cruise_alpha);
  void calcPCoeff(const double& cruise_speed, const double& cruise_alpha);
  void calcQCoeff(const double& cruise_speed, const double& cruise_alpha);
  void calcRCoeff(const double& cruise_speed, const double& cruise_alpha);
  void calcControlCoeff(const double& cruise_speed);  // TODO : consider cruise_alpha effect
  void writeResults();
};
}  // namespace gb
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
