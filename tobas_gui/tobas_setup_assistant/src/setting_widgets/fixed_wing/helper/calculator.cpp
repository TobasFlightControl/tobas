#include <tobas_setup_assistant/setting_tabs/fixed_wing/helper/calculator.hpp>

#include <format>
#include <iostream>

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
Calculator::Calculator(
  const kdl::Tree& tree,
  cf::CoefficientsWidget* coefs,
  WingsWidget* wings,
  ControlSurfacesWidget* control_surfaces)
  : super("Calculate")
  , tree_(tree)
  , inertia_solver_(tree)
  , coefs_(coefs)
  , wings_(wings)
  , control_surfaces_(control_surfaces)
{
  setFixedSize(kButtonWidth, kButtonHeight);

  connect(this, &QPushButton::clicked, this, &Calculator::onClicked);
}

void Calculator::updateInternalDataStructures()
{
  inertia_solver_.updateInternalDataStructures();

  // Center of gravity and inertia tensor.
  kdl::JntArray q(tree_.getNrOfJoints());
  if (inertia_solver_.jntToCart(q) < 0) {
    std::cout << inertia_solver_.errorMessage() << std::endl;
  }
  const auto& I_base = inertia_solver_.getInertia();
  mass_ = I_base.getMass();
  B_Pos_B2R_ = I_base.getCOG();
}

void Calculator::onClicked()
{
  // cruise飛行時の迎角, 速度を求める
  const auto cruise_speed = wings_->cruiseSpeed();
  const auto main_wing = wings_->getMainWing();
  const auto C_L =
    mass_ * st::kGravity / (dynamicPressure(st::kStandardAirDensity, cruise_speed) * main_wing->surfaceArea());
  const auto cruise_alpha = (C_L - main_wing->c_lift_0()) / main_wing->c_lift_alpha();
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
  // control surfacesに関して数値微分を取る
  calcControlCoeff(cruise_speed);
  writeResults();
}

void Calculator::XZ2DL(const kdl::Vector& force, const double& alpha, double& drag, double& lift)
{
  drag = -cos(alpha) * force.x() - sin(alpha) * force.z();
  lift = sin(alpha) * force.x() - cos(alpha) * force.z();
}

void Calculator::calcWingTranslations(const WingWidget* wing, kdl::Frame& W_T_R, kdl::Frame& R_T_W) const
{
  // base_link座標系 : urdfのbase_linkの座標系. flu座標系
  // wing座標系 : WingWidgetで設定した翼の, 翼根の前縁部を原点とする座標系. flu座標系
  // ref座標系 : この点のまわりで線形化した空力微係数を求める座標系. frd座標系. CoGに取る

  // GUIで設定するのは
  // base_link座標系でみたときのwingまでの位置と回転
  // base_link座標系でみたときのrefまでの位置 (ref座標系がfrd座標系であることは暗黙の了解)
  // ref座標系でみたときのwing座標系までのtransition (rotation + position)を求める
  const kdl::Vector B_Pos_B2W = wing->position();  // base_link座標系でみたときのbase_linkからwingまでのposition
  const kdl::Vector rpy = wing->rotation();
  const auto B_R_B2W =
    kdl::Rotation::RPY(rpy.x(), rpy.y(), rpy.z());  // base_link座標系でみたときのbase_linkからwingまでのrotation
  const auto B_T_W = kdl::Frame(B_R_B2W, B_Pos_B2W);  // base座標系で表したwing座標系のFrame
  const auto B_R_B2R =
    kdl::Rotation::RPY(M_PI, 0, 0);  // base_link座標系からみたときのbase_linkからref座標系までのrotation
  const auto B_T_R = kdl::Frame(B_R_B2R, B_Pos_B2R_);  // base_link座標系で表したref座標系のFrame
  W_T_R = B_T_W.inverse() * B_T_R;                    // wing座標系で表したref座標系のFrame
  R_T_W = B_T_R.inverse() * B_T_W;                    // ref座標系で表したwing座標系のFrame
}

void Calculator::calcMacTranslations(const WingWidget* wing, kdl::Frame& M_T_R, kdl::Frame& R_T_M) const
{
  // base_link座標系 : urdfのbase_linkの座標系. flu座標系
  // wing座標系 : WingWidgetで設定した翼の, 翼根の前縁部を原点とする座標系. flu座標系
  // mac座標系 : WingWidgetで設定した翼の, 空力中心を原点とする座標系. frd座標系.
  // wingがsymmetricか否かでwing座標系での位置が変わる
  // ref座標系 : この点のまわりで線形化した空力微係数を求める座標系. frd座標系. CoGに取る

  const auto W_T_M = kdl::Frame(
    kdl::Rotation::RPY(M_PI, 0, 0), kdl::Vector(wing->mac_position()));  // wing座標系で表したmac座標系のFrame
  kdl::Frame W_T_R, R_T_W;
  calcWingTranslations(wing, W_T_R, R_T_W);
  M_T_R = W_T_M.inverse() * W_T_R;
  R_T_M = R_T_W * W_T_M;
}

kdl::Wrench Calculator::calcMachineAeroDynamicForce(const kdl::Twist& twist_ref) const
{
  kdl::Wrench wrench_all = kdl::Wrench::Zero();
  for (int i = 0; i < wings_->count(); i++) {
    const auto wing = wings_->getWing(i);
    kdl::Frame M_T_R;  // mac座標系で表したref座標系のFrame
    kdl::Frame R_T_M;  // ref座標系で表したmac座標系のFrame
    calcMacTranslations(wing, M_T_R, R_T_M);

    // wing座標系で見た力とモーメント
    // twist_refはref座標系でみた これをmac座標系でみたtwist_wingを求める
    const auto twist_mac = M_T_R * twist_ref;
    const auto wrench_mac = calcWingAeroDynamicForce(wing, twist_mac);

    // ref座標系で見た力とモーメントを求める
    // ref座標系でみたときのmac座標系までのtransition (rotation + position)を用いる
    const auto wrench_ref = R_T_M * wrench_mac;  // ref座標系でみた力とモーメント
    wrench_all += wrench_ref;
  }
  return wrench_all;
}

kdl::Wrench Calculator::calcWingAeroDynamicForce(const WingWidget* wing, const kdl::Twist& twist_mac) const
{
  const auto vel_frd = twist_mac.vel.data;  // mac座標系はfrd座標系なのでこのまま計算
  const auto rot_frd = twist_mac.rot.data;

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
  const kdl::Vector lift =
    q * wing->surfaceArea() * C_L * kdl::Vector(sin(alpha), 0, -cos(alpha));  // mac座標系(frd)でみたときの力のベクトル
  const kdl::Vector drag = q * wing->surfaceArea() * C_D * kdl::Vector(-cos(alpha), 0, -sin(alpha));
  const auto force = lift + drag;
  const auto moment =
    kdl::Vector(q * wing->span() * C_l, q * wing->c_mac() * wing->c_pitch_0(), q * wing->span() * C_n);
  return kdl::Wrench(force, moment);
}

void Calculator::calcCruiseCoeff(const double& cruise_speed, const double& cruise_alpha)
{
  const auto vel = kdl::Vector(cruise_speed * cos(cruise_alpha), 0, cruise_speed * sin(cruise_alpha));
  kdl::Vector rot(0, 0, 0);    // ref座標系(frd)での回転角速度
  kdl::Twist twist(vel, rot);  // ref座標系(frd)でのtwist
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
  kdl::Vector rot(0, 0, 0);              // ref座標系(frd)での回転角速度
  kdl::Twist twist_plus(vel_plus, rot);  // ref座標系(frd)でのtwist
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
  const auto vel_plus =
    kdl::Vector(cruise_speed * cos(cruise_alpha), cruise_speed * sin(kDelta), cruise_speed * sin(cruise_alpha));
  const auto vel_minus =
    kdl::Vector(cruise_speed * cos(cruise_alpha), -cruise_speed * sin(kDelta), cruise_speed * sin(cruise_alpha));
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
  kdl::Vector rot_plus(kDelta * 2.0 * cruise_speed / span, 0, 0);  // ref座標系(frd)での回転角速度
  kdl::Vector rot_minus(-kDelta * 2.0 * cruise_speed / span, 0, 0);
  kdl::Twist twist_plus(vel, rot_plus);  // ref座標系(frd)でのtwist
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
  kdl::Vector rot_plus(0, kDelta * 2.0 * cruise_speed / c_mac, 0);  // ref座標系(frd)での回転角速度
  kdl::Vector rot_minus(0, -kDelta * 2.0 * cruise_speed / c_mac, 0);
  kdl::Twist twist_plus(vel, rot_plus);  // ref座標系(frd)でのtwist
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
  kdl::Vector rot_plus(0, 0, kDelta * 2.0 * cruise_speed / span);  // ref座標系(frd)での回転角速度
  kdl::Vector rot_minus(0, 0, -kDelta * 2.0 * cruise_speed / span);
  kdl::Twist twist_plus(vel, rot_plus);  // ref座標系(frd)でのtwist
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

void Calculator::calcControlCoeff(const double& cruise_speed)
{
  const auto n_of_cs = control_surfaces_->rowCount();
  control_surface_coefs_.resize(n_of_cs);
  const auto main_wing = wings_->getMainWing();
  for (int i = 0; i < n_of_cs; i++) {
    const auto wing = wings_->getWing(control_surfaces_->wingIdx(i));
    // translations
    kdl::Frame W_T_R;  // wing座標系で表したref座標系のFrame
    kdl::Frame R_T_W;  // ref座標系で表したwing座標系のFrame
    calcWingTranslations(wing, W_T_R, R_T_W);
    // control surface properties
    const auto c_root = wing->c_root();
    const auto c_tip = wing->c_tip();
    const auto k1 = control_surfaces_->startSpan(i);
    const auto k2 = control_surfaces_->finishSpan(i);
    CsCoefs coefs;
    kdl::Wrench wrench_ref;  // ref座標系(frd)でのwrench
    if (k1 >= 0) {
      // control surface前方のみ切り取った場合の翼の諸元
      const auto c_root_c = (1 - k1) * c_root + k1 * c_tip;
      const auto c_tip_c = (1 - k2) * c_root + k2 * c_tip;
      const auto span_c = (k2 - k1) * 0.5 * wing->span();
      const auto S_c = 0.5 * (c_root_c + c_tip_c) * span_c;
      // 翼空力中心にかかる力
      // mac座標系はfrd座標系とする
      const auto r = control_surfaces_->chordRatio(i);
      const auto c_ldelta = wing->c_lift_alpha() * (1 - 1 / M_PI * acos(2.0 * r - 1)) * kDelta;
      const auto lift = dynamicPressure(st::kStandardAirDensity, cruise_speed) * S_c * c_ldelta;
      const kdl::Wrench wrench_mac(kdl::Vector(0, 0, -lift), kdl::Vector(0, 0, 0));  // mac座標系でみたときのwrench
      // 翼座標系からみたときの翼空力中心の座標
      // 翼座標系はflu座標系
      const auto lambda = c_tip_c / c_root_c;
      const auto y_mac = -(lambda + 2) / (2 * lambda + 1) * 0.5 * span_c - k1 * 0.5 * wing->span();
      const auto x_mac = -(0.25 * c_root_c + y_mac * tan(wing->sweepBack()));
      const kdl::Vector W_Pos_W2M(x_mac, y_mac, 0);  // wing座標系でみたときのwingからmacまでのposition
      const auto W_T_M = kdl::Frame(kdl::Rotation::RPY(M_PI, 0, 0), W_Pos_W2M);  // wing座標系で表したmac座標系のFrame
      // ref座標系でみたときのwrenchに変換
      wrench_ref = R_T_W * W_T_M * wrench_mac;  // ref座標系で表したwrench
    }
    else if (k2 <= 0) {
      // control surface前方のみ切り取った場合の翼の諸元
      const auto c_root_c = (1 + k2) * c_root - k2 * c_tip;
      const auto c_tip_c = (1 + k1) * c_root - k1 * c_tip;
      const auto span_c = (-k1 + k2) * 0.5 * wing->span();
      const auto S_c = 0.5 * (c_root_c + c_tip_c) * span_c;
      // 翼空力中心にかかる力
      // mac座標系はfrd座標系とする
      const auto r = control_surfaces_->chordRatio(i);
      const auto c_ldelta = wing->c_lift_alpha() * (1 - 1 / M_PI * acos(2.0 * r - 1)) * kDelta;
      const auto lift = dynamicPressure(st::kStandardAirDensity, cruise_speed) * S_c * c_ldelta;
      const kdl::Wrench wrench_mac(kdl::Vector(0, 0, -lift), kdl::Vector(0, 0, 0));  // mac座標系でみたときのwrench
      // 翼座標系からみたときの翼空力中心の座標
      // 翼座標系はflu座標系
      const auto lambda = c_tip_c / c_root_c;
      const auto y_mac = (lambda + 2) / (2 * lambda + 1) * 0.5 * span_c - k2 * 0.5 * wing->span();
      const auto x_mac = -(0.25 * c_root_c - y_mac * tan(wing->sweepBack()));
      const kdl::Vector W_Pos_W2M(x_mac, y_mac, 0);  // wing座標系でみたときのwingからmacまでのposition
      const auto W_T_M = kdl::Frame(kdl::Rotation::RPY(M_PI, 0, 0), W_Pos_W2M);  // wing座標系で表したmac座標系のFrame
      // ref座標系でみたときのwrenchに変換
      wrench_ref = R_T_W * W_T_M * wrench_mac;  // ref座標系で表したwrench
    }
    else {
      // k1 ~ 0, 0 ~ k2に分けて計算
      kdl::Wrench wrench_left_ref, wrench_right_ref;
      // k1 ~ 0
      {
        const auto c_root_c = c_root;
        const auto c_tip_c = (1 + k1) * c_root - k1 * c_tip;
        const auto span_c = -k1 * 0.5 * wing->span();
        const auto S_c = 0.5 * (c_root_c + c_tip_c) * span_c;
        // 翼空力中心にかかる力
        // mac座標系はfrd座標系とする
        const auto r = control_surfaces_->chordRatio(i);
        const auto c_ldelta = wing->c_lift_alpha() * (1 - 1 / M_PI * acos(2.0 * r - 1)) * kDelta;
        const auto lift = dynamicPressure(st::kStandardAirDensity, cruise_speed) * S_c * c_ldelta;
        const kdl::Wrench wrench_mac(kdl::Vector(0, 0, -lift), kdl::Vector(0, 0, 0));  // mac座標系でみたときのwrench
        // 翼座標系からみたときの翼空力中心の座標
        // 翼座標系はflu座標系
        const auto lambda = c_tip_c / c_root_c;
        const auto y_mac = (lambda + 2) / (2 * lambda + 1) * 0.5 * span_c - k1 * 0.5 * wing->span();
        const auto x_mac = -(0.25 * c_root_c - y_mac * tan(wing->sweepBack()));
        const kdl::Vector W_Pos_W2M(x_mac, y_mac, 0);  // wing座標系でみたときのwingからmacまでのposition
        const auto W_T_M = kdl::Frame(kdl::Rotation::RPY(M_PI, 0, 0), W_Pos_W2M);  // wing座標系で表したmac座標系のFrame
        // ref座標系でみたときのwrenchに変換
        wrench_left_ref = R_T_W * W_T_M * wrench_mac;  // ref座標系で表したwrench
      }
      // 0 ~ k2
      {
        const auto c_root_c = c_root;
        const auto c_tip_c = (1 - k2) * c_root + k2 * c_tip;
        const auto span_c = k2 * 0.5 * wing->span();
        const auto S_c = 0.5 * (c_root_c + c_tip_c) * span_c;
        // 翼空力中心にかかる力
        // mac座標系はfrd座標系とする
        const auto r = control_surfaces_->chordRatio(i);
        const auto c_ldelta = wing->c_lift_alpha() * (1 - 1 / M_PI * acos(2.0 * r - 1)) * kDelta;
        const auto lift = dynamicPressure(st::kStandardAirDensity, cruise_speed) * S_c * c_ldelta;
        const kdl::Wrench wrench_mac(kdl::Vector(0, 0, -lift), kdl::Vector(0, 0, 0));  // mac座標系でみたときのwrench
        // 翼座標系からみたときの翼空力中心の座標
        // 翼座標系はflu座標系
        const auto lambda = c_tip_c / c_root_c;
        const auto y_mac = -(lambda + 2) / (2 * lambda + 1) * 0.5 * span_c - k2 * 0.5 * wing->span();
        const auto x_mac = -(0.25 * c_root_c + y_mac * tan(wing->sweepBack()));
        const kdl::Vector W_Pos_W2M(x_mac, y_mac, 0);  // wing座標系でみたときのwingからmacまでのposition
        const auto W_T_M = kdl::Frame(kdl::Rotation::RPY(M_PI, 0, 0), W_Pos_W2M);  // wing座標系で表したmac座標系のFrame
        // ref座標系でみたときのwrenchに変換
        wrench_right_ref = R_T_W * W_T_M * wrench_mac;  // ref座標系で表したwrench
      }
      wrench_ref = wrench_left_ref + wrench_right_ref;
    }
    // numerical differentiation
    coefs.drag = -wrench_ref.force.x() /
                 (dynamicPressure(st::kStandardAirDensity, cruise_speed) * main_wing->surfaceArea()) /
                 kDelta;  // alpha傾けてないのが怪しいがそもそもwrench_macの計算時に傾きを考えてないので近似的にはok
    coefs.lift = -wrench_ref.force.z() /
                 (dynamicPressure(st::kStandardAirDensity, cruise_speed) * main_wing->surfaceArea()) / kDelta;
    coefs.side = wrench_ref.force.y() /
                 (dynamicPressure(st::kStandardAirDensity, cruise_speed) * main_wing->surfaceArea()) / kDelta;
    coefs.roll =
      wrench_ref.torque.x() /
      (dynamicPressure(st::kStandardAirDensity, cruise_speed) * main_wing->surfaceArea() * main_wing->span()) / kDelta;
    coefs.pitch =
      wrench_ref.torque.y() /
      (dynamicPressure(st::kStandardAirDensity, cruise_speed) * main_wing->surfaceArea() * main_wing->c_mac()) / kDelta;
    coefs.yaw =
      wrench_ref.torque.z() /
      (dynamicPressure(st::kStandardAirDensity, cruise_speed) * main_wing->surfaceArea() * main_wing->span()) / kDelta;
    control_surface_coefs_[i] = coefs;
  }
  for (int i = 0; i < n_of_cs; i++) {
    const auto coefs = control_surface_coefs_[i];
    std::cout << "control surface No : " << i << std::endl;
    std::cout << "drag : " << coefs.drag << std::endl;
    std::cout << "lift : " << coefs.lift << std::endl;
    std::cout << "side : " << coefs.side << std::endl;
    std::cout << "roll : " << coefs.roll << std::endl;
    std::cout << "pitch: " << coefs.pitch << std::endl;
    std::cout << "yaw  : " << coefs.yaw << std::endl;
  }
}

void Calculator::writeResults()
{
  auto aero_coefs = coefs_->aeroCoefs();
  aero_coefs->c_lift_0(c_lift_0_);
  aero_coefs->c_lift_alpha(c_lift_alpha_);
  aero_coefs->c_drag_0(c_drag_0_);
  aero_coefs->c_drag_alpha(c_drag_alpha_);
  // TODO: c_drag_alpha^2
  aero_coefs->c_side_beta(c_side_beta_);
  // TODO: c_side_p, c_side_r
  aero_coefs->c_roll_beta(c_roll_beta_);
  aero_coefs->c_roll_p(c_roll_p_);
  aero_coefs->c_roll_r(c_roll_r_);
  aero_coefs->c_pitch_0(c_pitch_0_);
  aero_coefs->c_pitch_alpha(c_pitch_alpha_);
  aero_coefs->c_pitch_abs_beta(c_pitch_abs_beta_);
  aero_coefs->c_pitch_alpha_rate(c_pitch_alpha_rate_);
  aero_coefs->c_pitch_q(c_pitch_q_);
  aero_coefs->c_yaw_beta(c_yaw_beta_);
  aero_coefs->c_yaw_p(c_yaw_p_);
  aero_coefs->c_yaw_r(c_yaw_r_);

  auto cs_widget = coefs_->controlSurfaces();
  for (int i = 0; i < control_surfaces_->rowCount(); i++) {
    cs_widget->liftCoef(i, control_surface_coefs_[i].lift);
    cs_widget->dragCoef(i, control_surface_coefs_[i].drag);
    cs_widget->sideCoef(i, control_surface_coefs_[i].side);
    cs_widget->rollCoef(i, control_surface_coefs_[i].roll);
    cs_widget->pitchCoef(i, control_surface_coefs_[i].pitch);
    cs_widget->yawCoef(i, control_surface_coefs_[i].yaw);
  }

  qt::qInfoBox(this, "Coefficients are estimated successfully.");
}
}  // namespace hp
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
