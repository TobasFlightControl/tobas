#include <tobas_setup_assistant/setting_tabs/fixed_wing/helper/calculator.hpp>

#include <iostream>
#include <format>

#include <tobas_kdl/conversion/coordinates.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/fixed_wing.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace hp
{
Calculator::Calculator(const kdl::Tree& tree, VehicleParametersWidget* vehicle, WingsWidget* wings, ControlSurfacesWidget* control_surfaces): super("Calculate"), mass_holder_(tree), vehicle_(vehicle), wings_(wings), control_surfaces_(control_surfaces)
{
  setFixedSize(kButtonWidth, kButtonHeight);

  connect(this, &QPushButton::clicked, this, &Calculator::onClicked);
}

void Calculator::updateInternalDataStructures()
{
  mass_holder_.updateInternalDataStructures();
}

void Calculator::onClicked()
{
  std::cout << "start" << std::endl;
  // cruise飛行時の迎角, 速度を求める
  const auto cruise_speed = wings_->cruiseSpeed();
  const auto mass = mass_holder_.getMass();
  const auto main_wing = wings_->getMainWing();
  const auto C_L = mass * st::kGravity / (dynamicPressure(st::kStandardAirDensity, cruise_speed) * main_wing->surfaceArea());
  std::cout << "C_L : " << C_L << std::endl;
  const auto cruise_alpha = (C_L - main_wing->c_lift_0()) / main_wing->c_lift_alpha();
  std::cout << "cruise_alpha : " << cruise_alpha / M_PI * 180.0 << " deg" << std::endl;
  validate(cruise_alpha);
  // cruise時係数を求める
  calcCruiseCoeff(cruise_speed, cruise_alpha);
  // alphaに関して数値微分を取る
  calcAoACoeff(cruise_speed, cruise_alpha);
  // betaに関して数値微分を取る
  calcAoSCoeff(cruise_speed, cruise_alpha);
  // pに関して数値微分を取る
  calcPCoeff(cruise_speed, cruise_alpha);
  // qに関して数値微分を取る
  calcQCoeff(cruise_speed, cruise_alpha);
  // rに関して数値微分を取る
  calcRCoeff(cruise_speed, cruise_alpha);
}

void Calculator::XZ2DL(const kdl::Vector& force, const double& alpha, double& drag, double& lift)
{
  drag = -cos(alpha) * force.x() - sin(alpha) * force.z();
  lift = sin(alpha) * force.x() - cos(alpha) * force.z();
}

void Calculator::validate(const double& cruise_alpha)
{
  const auto main_wing = wings_->getMainWing();
  // wing surface
  if (abs(main_wing->surfaceArea() - vehicle_->wingSurface()) / vehicle_->wingSurface() > kAffordableRate) {
    qt::qWarnBox(this, QString::fromStdString(std::format("The wing area entered in the Vehicle Parameters section differs significantly from the estimated wing area.\nEstimated : {:.2f} m^2", main_wing->surfaceArea())));
  }
  if (abs(main_wing->span() - vehicle_->wingSpan()) / vehicle_->wingSpan() > kAffordableRate) {
    qt::qWarnBox(this, QString::fromStdString(std::format("The wing span entered in the Vehicle Parameters section differs significantly from the estimated wing area.\nEstimated : {:.2f} m", main_wing->span())));
  }
  if (abs(main_wing->c_mac() - vehicle_->mac()) / vehicle_->mac() > kAffordableRate) {
    qt::qWarnBox(this, QString::fromStdString(std::format("The mean aerodynamics chord entered in the Vehicle Parameters section differs significantly from the estimated wing area.\nEstimated : {:.2f} m", main_wing->c_mac())));
  }
  if (!vehicle_->alphaLimit().inRange(cruise_alpha)) {
    qt::qWarnBox(this, QString::fromStdString(std::format("The cruise AoA is outside the valid range entered in the Vehicle Parameters section.\nEstimated cruise AoA : {:2f} rad", cruise_alpha)));
  }
}

kdl::Wrench Calculator::calcMachineAeroDynamicForce(const kdl::Twist& twist_ref) const
{
  kdl::Wrench wrench_all = kdl::Wrench::Zero();
  for (int i = 0; i < wings_->count(); i++) {
    const auto wing = wings_->getWing(i);
    // GUIで設定するのは
    // base_link座標系でみたときのwingまでの位置と回転
    // base_link座標系でみたときのrefまでの位置 (ref座標系がfrd座標系であることは暗黙の了解)
    // ref座標系でみたときのwing座標系までのtransition (rotation + position)を求める
    const kdl::Vector B_Pos_B2W = wing->position(); // base_link座標系でみたときのbase_linkからwingまでのposition
    const kdl::Vector rpy = wing->rotation();
    const auto B_R_B2W = kdl::Rotation::RPY(rpy.x(), rpy.y(), rpy.z()); // base_link座標系でみたときのbase_linkからwingまでのrotation
    const auto B_T_W = kdl::Frame(B_R_B2W, B_Pos_B2W); // base座標系で表したwing座標系のFrame
    const kdl::Vector B_Pos_B2R = vehicle_->aerodynamicCenter(); // base_link座標系からみたときのbase_linkからref座標系までのposition
    const auto B_R_B2R = kdl::Rotation::RPY(M_PI, 0, 0); // base_link座標系からみたときのbase_linkからref座標系までのrotation
    const auto B_T_R = kdl::Frame(B_R_B2R, B_Pos_B2R); // base_link座標系で表したref座標系のFrame
    const auto W_T_R = B_T_W.inverse() * B_T_R; // wing座標系で表したref座標系のFrame
    const auto R_T_W = B_T_R.inverse() * B_T_W; // ref座標系で表したwing座標系のFrame

    // wing座標系で見た力とモーメント
    // twist_refはref座標系でみた これをwing座標系でみたtwist_wingを求める
    const auto twist_wing = W_T_R * twist_ref;
    const auto wrench_wing = calcWingAeroDynamicForce(wing, twist_wing);

    // ref座標系で見た力とモーメントを求める
    // ref座標系でみたときのwing座標系までのtransition (rotation + position)を用いる
    const auto wrench_ref = R_T_W * wrench_wing; // ref座標系でみた力とモーメント
    wrench_all += wrench_ref;
  }
  return wrench_all;
}

kdl::Wrench Calculator::calcWingAeroDynamicForce(const WingWidget* wing, const kdl::Twist& twist_frd) const
{
  const auto vel_frd = twist_frd.vel.data;
  const auto rot_frd = twist_frd.rot.data;

  const auto airspeed = vel_frd.norm();
  const auto alpha = angleOfAttack(vel_frd);
  const auto beta = angleOfSideSlip(vel_frd);
  const auto p = rot_frd.x();
  const auto r = rot_frd.z();

  const auto C_L = wing->c_lift(alpha);
  const auto C_D = wing->c_drag(alpha);
  const auto C_l = wing->c_roll(airspeed, alpha, beta, p, r);
  const auto C_n = wing->c_yaw(airspeed, alpha, p, r);

  const auto q = dynamicPressure(st::kStandardAirDensity, airspeed);
  // std::cout << "q : " << q << std::endl;
  // std::cout << "c_tip : " << wing->c_tip() << std::endl;
  // std::cout << "c_root : " << wing->c_root() << std::endl;
  // std::cout << "span : " << wing->span() << std::endl;
  // std::cout << "S : " << wing->surfaceArea() << std::endl;
  const kdl::Vector lift = q * wing->surfaceArea() * C_L * kdl::Vector(sin(alpha), 0, -cos(alpha));
  const kdl::Vector drag = q * wing->surfaceArea() * C_D * kdl::Vector(-cos(alpha), 0, -sin(alpha));
  // std::cout << "lift : " << lift << std::endl;
  // std::cout << "drag : " << drag << std::endl;
  const auto force = lift + drag;
  const auto moment = kdl::Vector(q * wing->span() * C_l, q * wing->c_mac() * wing->c_pitch_0(), q * wing->span() * C_n);
  // std::cout << "moment : " << moment << std::endl;
  return kdl::Wrench(force, moment);
}

void Calculator::calcCruiseCoeff(const double& cruise_speed, const double& cruise_alpha)
{
  const auto vel = kdl::Vector(cruise_speed * cos(cruise_alpha), 0, cruise_speed * sin(cruise_alpha));
  kdl::Vector rot(0, 0, 0); // ref座標系(frd)での回転角速度
  kdl::Twist twist(vel, rot); // ref座標系(frd)でのtwist
  const auto wrench = calcMachineAeroDynamicForce(twist);
  double D, L;
  XZ2DL(wrench.force, cruise_alpha, D, L);
  const auto q = dynamicPressure(st::kStandardAirDensity, cruise_speed);
  const auto S = wings_->getMainWing()->surfaceArea();
  const auto c_mac = wings_->getMainWing()->c_mac();
  c_lift_0_ = L / (q * S);
  c_drag_0_ = D / (q * S);
  c_pitch_0_ = wrench.torque.y() / (q * S * c_mac);
  std::cout << "c_lift_0 : " << c_lift_0_ << std::endl;
  std::cout << "c_drag_0 : " << c_drag_0_ << std::endl;
  std::cout << "c_pitch_0 : " << c_pitch_0_ << std::endl;
}

void Calculator::calcAoACoeff(const double& cruise_speed, const double& cruise_alpha)
{
  const auto alpha_plus = cruise_alpha + kDelta;
  const auto alpha_minus = cruise_alpha - kDelta;
  const auto vel_plus = kdl::Vector(cruise_speed * cos(alpha_plus), 0, cruise_speed * sin(alpha_plus));
  const auto vel_minus = kdl::Vector(cruise_speed * cos(alpha_minus), 0, cruise_speed * sin(alpha_minus));
  kdl::Vector rot(0, 0, 0); // ref座標系(frd)での回転角速度
  kdl::Twist twist_plus(vel_plus, rot); // ref座標系(frd)でのtwist
  kdl::Twist twist_minus(vel_minus, rot);
  // alpha plus
  const auto wrench_plus = calcMachineAeroDynamicForce(twist_plus);
  double D_plus, L_plus;
  XZ2DL(wrench_plus.force, alpha_plus, D_plus, L_plus);
  // alpha minus
  const auto wrench_minus = calcMachineAeroDynamicForce(twist_minus);
  double D_minus, L_minus;
  XZ2DL(wrench_minus.force, alpha_minus, D_minus, L_minus);
  // numerical differentiation
  const auto q = dynamicPressure(st::kStandardAirDensity, cruise_speed);
  const auto S = wings_->getMainWing()->surfaceArea();
  const auto c_mac = wings_->getMainWing()->c_mac();
  c_lift_alpha_ = (L_plus - L_minus) / (q * S) / (2.0 * kDelta);
  c_drag_alpha_ = (D_plus - D_minus) / (q * S) / (2.0 * kDelta);
  c_pitch_alpha_ = (wrench_plus.torque.y() - wrench_minus.torque.y()) / (q * S * c_mac) / (2.0 * kDelta);
  std::cout << "C_Lalpha : " << c_lift_alpha_ << std::endl;
  std::cout << "C_Dalpha : " << c_drag_alpha_ << std::endl;
  std::cout << "C_malpha : " << c_pitch_alpha_ << std::endl;
}

void Calculator::calcAoSCoeff(const double& cruise_speed, const double& cruise_alpha)
{
  const auto vel_plus = kdl::Vector(cruise_speed * cos(cruise_alpha), cruise_speed * sin(kDelta), cruise_speed * sin(cruise_alpha));
  const auto vel_minus = kdl::Vector(cruise_speed * cos(cruise_alpha), -cruise_speed * sin(kDelta), cruise_speed * sin(cruise_alpha));
  kdl::Vector rot(0, 0, 0);
  kdl::Twist twist_plus(vel_plus, rot);
  kdl::Twist twist_minus(vel_minus, rot);
  // beta plus
  const auto wrench_plus = calcMachineAeroDynamicForce(twist_plus);
  // beta minus
  const auto wrench_minus = calcMachineAeroDynamicForce(twist_minus);
  // numerical differentiation
  const auto q = dynamicPressure(st::kStandardAirDensity, cruise_speed);
  const auto S = wings_->getMainWing()->surfaceArea();
  const auto span = wings_->getMainWing()->span();
  c_side_beta_ = (wrench_plus.force.y() - wrench_minus.force.y()) / (q * S) / (2.0 * kDelta);
  c_roll_beta_ = (wrench_plus.torque.x() - wrench_minus.torque.x()) / (q * S * span) / (2.0 * kDelta);
  c_yaw_beta_ = (wrench_plus.torque.z() - wrench_minus.torque.z()) / (q * S * span) / (2.0 * kDelta);
  std::cout << "C_Ybeta : " << c_side_beta_ << std::endl;
  std::cout << "C_lbeta : " << c_roll_beta_ << std::endl;
  std::cout << "C_rbeta : " << c_yaw_beta_ << std::endl;
}

void Calculator::calcPCoeff(const double& cruise_speed, const double& cruise_alpha)
{
  const auto vel = kdl::Vector(cruise_speed * cos(cruise_alpha), 0, cruise_speed * sin(cruise_alpha));
  const auto span = wings_->getMainWing()->span();
  kdl::Vector rot_plus(kDelta * 2.0 * cruise_speed / span, 0, 0); // ref座標系(frd)での回転角速度
  kdl::Vector rot_minus(-kDelta * 2.0 * cruise_speed / span, 0, 0);
  kdl::Twist twist_plus(vel, rot_plus); // ref座標系(frd)でのtwist
  kdl::Twist twist_minus(vel, rot_minus);
  // p plus
  const auto wrench_plus = calcMachineAeroDynamicForce(twist_plus);
  // p minus
  const auto wrench_minus = calcMachineAeroDynamicForce(twist_minus);
  // numerical differentiation
  const auto q = dynamicPressure(st::kStandardAirDensity, cruise_speed);
  const auto S = wings_->getMainWing()->surfaceArea();
  c_side_p_ = (wrench_plus.force.y() - wrench_minus.force.y()) / (q * S) / (2.0 * kDelta);
  c_roll_p_ = (wrench_plus.torque.x() - wrench_minus.torque.x()) / (q * S * span) / (2.0 * kDelta);
  c_yaw_p_ = (wrench_plus.torque.z() - wrench_minus.torque.z()) / (q * S * span) / (2.0 * kDelta);
  std::cout << "C_Yp : " << c_side_p_ << std::endl;
  std::cout << "C_lp : " << c_roll_p_ << std::endl;
  std::cout << "C_np : " << c_yaw_p_ << std::endl;
}

void Calculator::calcQCoeff(const double& cruise_speed, const double& cruise_alpha)
{
  const auto vel = kdl::Vector(cruise_speed * cos(cruise_alpha), 0, cruise_speed * sin(cruise_alpha));
  const auto c_mac = wings_->getMainWing()->c_mac();
  kdl::Vector rot_plus(0, kDelta * 2.0 * cruise_speed / c_mac, 0); // ref座標系(frd)での回転角速度
  kdl::Vector rot_minus(0, -kDelta * 2.0 * cruise_speed / c_mac, 0);
  kdl::Twist twist_plus(vel, rot_plus); // ref座標系(frd)でのtwist
  kdl::Twist twist_minus(vel, rot_minus);
  // p plus
  const auto wrench_plus = calcMachineAeroDynamicForce(twist_plus);
  // p minus
  const auto wrench_minus = calcMachineAeroDynamicForce(twist_minus);
  // numerical differentiation
  const auto q = dynamicPressure(st::kStandardAirDensity, cruise_speed);
  const auto S = wings_->getMainWing()->surfaceArea();
  c_pitch_q_ = (wrench_plus.torque.y() - wrench_minus.torque.y()) / (q * S * c_mac) / (2.0 * kDelta);
  std::cout << "C_mq : " << c_pitch_q_ << std::endl;
}

void Calculator::calcRCoeff(const double& cruise_speed, const double& cruise_alpha)
{
  const auto vel = kdl::Vector(cruise_speed * cos(cruise_alpha), 0, cruise_speed * sin(cruise_alpha));
  const auto span = wings_->getMainWing()->span();
  kdl::Vector rot_plus(0, 0, kDelta * 2.0 * cruise_speed / span); // ref座標系(frd)での回転角速度
  kdl::Vector rot_minus(0, 0, -kDelta * 2.0 * cruise_speed / span);
  kdl::Twist twist_plus(vel, rot_plus); // ref座標系(frd)でのtwist
  kdl::Twist twist_minus(vel, rot_minus);
  // p plus
  const auto wrench_plus = calcMachineAeroDynamicForce(twist_plus);
  // p minus
  const auto wrench_minus = calcMachineAeroDynamicForce(twist_minus);
  // numerical differentiation
  const auto q = dynamicPressure(st::kStandardAirDensity, cruise_speed);
  const auto S = wings_->getMainWing()->surfaceArea();
  c_side_r_ = (wrench_plus.force.y() - wrench_minus.force.y()) / (q * S) / (2.0 * kDelta);
  c_roll_r_ = (wrench_plus.torque.x() - wrench_minus.torque.x()) / (q * S * span) / (2.0 * kDelta);
  c_yaw_r_ = (wrench_plus.torque.z() - wrench_minus.torque.z()) / (q * S * span) / (2.0 * kDelta);
  std::cout << "C_Yr : " << c_side_r_ << std::endl;
  std::cout << "C_lr : " << c_roll_r_ << std::endl;
  std::cout << "C_nr : " << c_yaw_r_ << std::endl;
}
}  // namespace hp
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
