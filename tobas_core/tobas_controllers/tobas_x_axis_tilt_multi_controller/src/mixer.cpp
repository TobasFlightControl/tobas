#include "tobas_x_axis_tilt_multi_controller/mixer.hpp"

#include <ranges>

#include <tobas_constants/constants.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_std_tools/universal_constants.hpp>

using namespace std;
using namespace Eigen;

namespace tobas
{
namespace x_axis_tilt_multicopter
{
Mixer::Mixer(const Drone& drone, const kdl::Tree& tree) : super(drone, tree), fk_solver_(tree), inertia_solver_(tree)
{
}

bool Mixer::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  const auto nr = drone_.prop->numRotors();

  E_.conservativeResize(NoChange, 2 * nr);
  for (int i = 0; i < nr; ++i) {
    E_.block<2, 2>(3, 2 * i).diagonal().setIdentity();
  }

  x_.conservativeResize(2 * nr);

  thrust_points_.clear();

  for (const auto& [idx, rotor_it] : views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;

    if (rotor->tilt_joint_name.empty()) {
      cerr << "The tilt joint of rotor " << rotor->link_name << " is not specified." << endl;
      return false;
    }

    const auto& cur_elem = tree_.getSegment(rotor->link_name)->second;
    const auto& cur_seg = cur_elem.segment;
    const auto& par_elem = cur_elem.parent->second;
    const auto& par_seg = par_elem.segment;
    const auto& par_joint = par_seg.joint();

    // 祖父母リンクから見た推力の作用点を保存
    const auto gpar_T_cur = par_seg.frame() * cur_seg.frame();
    const auto& rotor_pos = gpar_T_cur.p;
    const auto thrust_pos = eigen::projectPointOnToLine(par_joint.origin.data, par_joint.axis().data, rotor_pos.data);
    thrust_points_[rotor->link_name] = thrust_pos;

    // プロペラリンクとチルト軸の距離が十分に小さいことを保証
    // TODO: サイズや推力など，何らかの根拠に基づいて不安定にならない閾値を決める．
    const auto rotor_offset = rotor_pos - thrust_pos;
    const auto rotor_offset_norm = rotor_offset.norm();
    if (rotor_offset_norm > INFINITY) {
      cerr << "The distance between propeller \"" << rotor->link_name
           << "\" and its tilt axis is too large: " << rotor_offset_norm << endl;
      return false;
    }
  }

  return true;
}

bool Mixer::solve(
  const kdl::JntArray& cur_q,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_dgyro_B,
  const double& ux,
  const double& uz,
  const kdl::Vector& ext_torque_B)
{
  // 順運動学を計算
  if (fk_solver_.jntToCart(cur_q) < 0) {
    cerr << "Forward kinematics failed: " << fk_solver_.errorMessage() << endl;
    return false;
  }

  // 質量特性を計算
  if (inertia_solver_.jntToCart(cur_q) < 0) {
    cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << endl;
    return false;
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  for (const auto& [idx, rotor_it] : views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;

    // 祖父母フレームを取得
    const auto& cur_elem = tree_.getSegment(rotor->link_name)->second;
    const auto& par_elem = cur_elem.parent->second;
    const auto& gpar_elem = par_elem.parent->second;
    const auto& gpar_seg = gpar_elem.segment;
    const auto& B_T_gpar = fk_solver_.getFrame(gpar_seg.name());

    // 運動方程式の左辺を計算
    const auto col = 2 * idx;
    if (rotor_alive_.at(rotor->link_name)) {
      const auto B_Pos_B2P = B_T_gpar * thrust_points_.at(rotor->link_name);
      const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
      const auto d_cm = rotor->sign() * rotor->moment_const;
      E_(0, col) = -d_cm;
      E_(1, col) = B_Pos_G2P.z();
      E_(2, col) = -B_Pos_G2P.y();
      E_(0, col + 1) = B_Pos_G2P.y();
      E_(1, col + 1) = -B_Pos_G2P.x();
      E_(2, col + 1) = -d_cm;
    }
    else {
      // ロータが死んでいる時は推力から期待の運動への伝達をゼロにすることで最適推力がゼロになるよう仕向ける
      E_.middleCols<2>(col).setZero();
    }
  }

  // 運動方程式の右辺
  const auto eom_rot_right_B = I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B) - ext_torque_B;  // [Nm]
  f_.tail<3>() = eom_rot_right_B.data;

  // 推力和の条件
  f_(3) = ux;
  f_(4) = uz;

  // Ex = f の最小二乗解 (冗長自由度がある場合はxのL2ノルム最小化)
  // TODO: 推力の絶対値の制約を考慮．凸最適化問題にすれば良さそう．
  x_ = E_.jacobiSvd(ComputeThinU | ComputeThinV).solve(f_);

  return true;
}

double Mixer::getThrust(size_t idx) const
{
  return thrustDeadband(x_.segment<2>(2 * idx).norm());
}

double Mixer::getTiltAngle(size_t idx) const
{
  const auto tx = x_(2 * idx);
  const auto ty = x_(2 * idx + 1);
  return atan2(ty, tx);
}
}  // namespace x_axis_tilt_multicopter
}  // namespace tobas
