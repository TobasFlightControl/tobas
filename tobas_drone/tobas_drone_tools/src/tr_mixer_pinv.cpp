#include <ranges>

#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_drone_tools/tr_mixer_pinv.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
TiltRotorMixer_pinv::TiltRotorMixer_pinv(const Drone& drone, const kdl::Tree& tree)
  : drone_(drone), tree_(tree), fk_solver_(tree), inertia_solver_(tree)
{
  if (drone_.numRotors() > 0 && tree_.getNrOfJoints() > 0)
    if (!updateInternalDataStructures())
      PRINT_ERROR("Failed to update internal data structures of TiltRotorMixer_pinv.");
}

bool TiltRotorMixer_pinv::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  const auto nr = drone_.numRotors();

  A_.resize(nr);
  for (const auto& [idx, rotor_it] : views::enumerate(drone_.rotors))
  {
    const auto& rotor = rotor_it.second;

    if (rotor.tilt_joint_name.empty())
    {
      cerr << "The tilt joint of rotor " << rotor.link_name << " is not specified." << endl;
      return false;
    }

    const auto& cur_elem = tree_.getSegment(rotor.link_name)->second;
    const auto& cur_seg = cur_elem.segment;
    const auto& cur_joint = cur_seg.joint();
    const auto& par_elem = cur_elem.parent->second;
    const auto& par_seg = par_elem.segment;
    const auto& par_joint = par_seg.joint();

    const auto& p = par_joint.axis();                      // 祖父母リンクから見たティルト軸
    const auto& q = par_seg.frame().M * cur_joint.axis();  // 親リンクのジョイントフレームから見たロータ軸

    // TODO: ティルトジョイントがロータジョイントの直接の親じゃない場合にも対応
    if (par_joint.name != rotor.tilt_joint_name)
    {
      cerr << "Tilt joint name of rotor " << rotor.link_name << " mismatch." << endl;
      return false;
    }

    if (!p.isOrthogonal(q))
    {
      cerr << "The axes of tilt joint " << par_joint.name << " and rotor joint " << cur_joint.name
           << " must be orthogonal." << endl;
      return false;
    }

    // TODO: プロペラリンクとティルト軸の距離が閾値以下であることを保証

    A_.at(idx).col(0) = q.data;
    A_.at(idx).col(1) = (p * q).data;
  }

  E_.conservativeResize(NoChange, 2 * nr);
  x_.conservativeResize(2 * nr);

  is_singular_.resize(nr, false);

  return true;
}

bool TiltRotorMixer_pinv::solve(
  const kdl::JntArray& cur_q,
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_acc_W,
  const kdl::Vector& tar_dgyro_B)
{
  // 順運動学を計算
  if (fk_solver_.JntToCart(cur_q) < 0)
  {
    cerr << "Forward kinematics failed: " << fk_solver_.errorMessage() << endl;
    return false;
  }

  // 質量特性を計算
  if (inertia_solver_.JntToCart(cur_q) < 0)
  {
    cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << endl;
    return false;
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();
  const auto& mass = inertia.getMass();

  for (const auto& [idx, rotor_it] : views::enumerate(drone_.rotors))
  {
    const auto& rotor = rotor_it.second;

    // ロータリンクとその親要素を取得
    const auto& cur_elem = tree_.getSegment(rotor.link_name)->second;
    const auto& par_elem = cur_elem.parent->second;
    const auto& gpar_elem = par_elem.parent->second;

    // 祖父母フレームの姿勢行列を取得
    const auto& gpar_seg = gpar_elem.segment;
    const auto& B_Rot_gpar = fk_solver_.getFrame(gpar_seg.name()).M;

    // ティルト軸と鉛直方向の偏角を計算
    const auto& par_seg = par_elem.segment;
    const auto& par_joint = par_seg.joint();
    const auto tilt_axis_B = B_Rot_gpar * par_joint.axis();
    const auto tilt_axis_W = cur_rot * tilt_axis_B;
    auto declination = tilt_axis_W.argument(kdl::Vector::UnitZ());
    if (declination > M_PI_2)
      declination = M_PI - declination;

    // 特異状態を更新
    if (is_singular_[idx])
    {
      if (declination > tobas_std::deg2rad(kSingularThreshUB))
        is_singular_[idx] = false;
    }
    else
    {
      if (declination < tobas_std::deg2rad(kSingularThreshLB))
        is_singular_[idx] = true;
    }

    // 運動方程式の左辺を計算
    const auto col = 2 * idx;
    if (is_singular_[idx])
    {
      // 特異状態の時は推力から期待の運動への伝達をゼロにすることで最適推力がゼロになるよう仕向ける
      E_.block<3, 2>(0, col).setZero();
      E_.block<3, 2>(3, col).setZero();
    }
    {
      const auto& B_Pos_B2P = fk_solver_.getFrame(rotor.link_name).p;
      const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;

      const auto d = rotor.sign();
      const auto& cm = rotor.moment_constant;

      const Matrix<double, 3, 2> B = B_Rot_gpar.data * A_.at(idx);
      const Matrix3d C = eigen::skew(B_Pos_G2P.data) - (d * cm) * Diagonal3d(1, 1, 1);
      const auto D = C * B;

      E_.block<3, 2>(0, col) = B;
      E_.block<3, 2>(3, col) = D;
    }
  }

  // 運動方程式の右辺を計算
  const kdl::Vector grav_W(0, 0, -tobas_std::kGravity);
  f_.head<3>() = (mass * cur_rot.inverse(tar_acc_W - grav_W)).data;
  f_.tail<3>() = (I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B)).data;

  // Ex = f の最小二乗解を求める
  // 冗長自由度がある場合はxのL2ノルムを最小化する
  // TODO: 推力の絶対値の制約を考慮．凸最適化問題にすれば良さそう．
  x_ = E_.jacobiSvd(ComputeThinU | ComputeThinV).solve(f_);

  // 特異状態に対応する解を最小値に固定
  for (const auto& [idx, rotor_it] : views::enumerate(drone_.rotors))
  {
    const auto& rotor = rotor_it.second;
    if (is_singular_[idx])
    {
      x_(2 * idx) = rotor.minThrust();
      x_(2 * idx + 1) = 0.;
    }
  }

  return true;
}

double TiltRotorMixer_pinv::getThrust(size_t idx) const
{
  return x_.segment<2>(2 * idx).norm();
}

double TiltRotorMixer_pinv::getTiltAngle(size_t idx) const
{
  const auto tx = x_(2 * idx);
  const auto ty = x_(2 * idx + 1);
  return atan2(ty, tx);
}
}  // namespace tobas
