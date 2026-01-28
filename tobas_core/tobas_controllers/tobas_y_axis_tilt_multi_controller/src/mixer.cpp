#include "tobas_y_axis_tilt_multi_controller/mixer.hpp"

#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_math/float.hpp>
#include <tobas_std_tools/universal_constants.hpp>

using namespace std;
using namespace Eigen;

namespace tobas
{
namespace y_axis_tilt_multicopter
{
Mixer::Mixer(const Drone& drone, const kdl::Tree& tree) : super(drone, tree), fk_solver_(tree), inertia_solver_(tree)
{
}

bool Mixer::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  if (!fk_solver_.updateInternalDataStructures()) {
    cerr << fk_solver_.errorMessage() << endl;
    return false;
  }
  if (!inertia_solver_.updateInternalDataStructures()) {
    cerr << inertia_solver_.errorMessage() << endl;
    return false;
  }

  // 順運動学を計算
  if (fk_solver_.jntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints())) < 0) {
    cerr << fk_solver_.errorMessage() << endl;
    return false;
  }

  const auto nr = drone_.prop->numRotors();

  E_.conservativeResize(NoChange, 2 * nr);
  for (size_t i = 0; i < nr; ++i) {
    E_.block<2, 2>(3, 2 * i).setIdentity();
  }
  x_.conservativeResize(2 * nr);

  thrust_points_.resize(nr);
  tilt_axis_signs_.resize(nr);
  tilt_offsets_.resize(nr);

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
    const auto& gpar_elem = par_elem.parent->second;
    const auto& gpar_seg = gpar_elem.segment;

    // 祖父母リンクから見た推力の作用点を保存
    const auto gpar_T_cur = par_seg.frame() * cur_seg.frame();
    const auto& rotor_pos = gpar_T_cur.p;
    const auto thrust_pos = eigen::projectPointOnToLine(par_joint.origin.data, par_joint.axis().data, rotor_pos.data);
    thrust_points_.at(idx) = thrust_pos;

    // チルト軸の符号を保存
    const auto& B_T_gpar = fk_solver_.getFrame(gpar_seg.name());
    const auto tilt_axis = B_T_gpar.M * par_joint.axis();  // ベースリンクから見たチルト軸
    const auto tilt_axis_y = tilt_axis.normalized().y();
    if (!math::isClose(std::abs(tilt_axis_y), 1.)) {
      cerr << "Tilt axis must be parallel to the Y axis." << endl;
      return false;
    }
    tilt_axis_signs_.at(idx) = math::sign(tilt_axis_y);

    // チルトジョイントの角度が0のときのチルト角の垂直上方向からのオフセットを保存
    // FIXME: 機体自体がY軸周りに折れ曲がる場合はこのオフセットが変化する！
    const auto& B_T_par = fk_solver_.getFrame(par_elem.segment.name());
    const auto n = B_T_par.M * cur_elem.segment.joint().axis();  // ベースリンクから見た回転軸
    if (!math::isClose(n.y(), 0.)) {
      cerr << "The Y component of the propeller’s axis of rotation must be zero." << endl;
      return false;
    }
    tilt_offsets_.at(idx) = atan2(n.x(), n.z());
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
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  for (const auto& [idx, rotor_it] : views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;

    const auto& cur_elem = tree_.getSegment(rotor->link_name)->second;
    const auto& par_elem = cur_elem.parent->second;
    const auto& gpar_elem = par_elem.parent->second;

    // 運動方程式の左辺を計算
    const auto col_tx = 2 * idx;
    const auto col_tz = col_tx + 1;
    if (rotor_alive_.at(rotor->link_name)) {
      const auto& B_T_gpar = fk_solver_.getFrame(gpar_elem.segment.name());
      const auto B_Pos_B2P = B_T_gpar * thrust_points_.at(idx);
      const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
      const auto d_cm = rotor->sign() * rotor->momentConst();
      E_(0, col_tx) = -d_cm;
      E_(1, col_tx) = B_Pos_G2P.z();
      E_(2, col_tx) = -B_Pos_G2P.y();
      E_(0, col_tz) = B_Pos_G2P.y();
      E_(1, col_tz) = -B_Pos_G2P.x();
      E_(2, col_tz) = -d_cm;
    }
    else {
      // ロータが死んでいる時は推力から機体の運動への伝達をゼロにすることで最適推力がゼロになるよう仕向ける
      E_.middleCols<2>(col_tx).setZero();
    }
  }

  // 運動方程式の右辺
  const auto eom_rot_right_B = I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B) - ext_torque_B;  // [Nm]
  f_.head<3>() = eom_rot_right_B.data;

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
  const auto& tx = x_(2 * idx);
  const auto& tz = x_(2 * idx + 1);
  return tilt_axis_signs_.at(idx) * algo::wrapPi(atan2(tx, tz) - tilt_offsets_.at(idx));
}
}  // namespace y_axis_tilt_multicopter
}  // namespace tobas
