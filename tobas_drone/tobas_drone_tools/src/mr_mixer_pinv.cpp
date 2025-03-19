#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_drone_tools/mr_mixer_pinv.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
MultiRotorMixer_pinv::MultiRotorMixer_pinv(const Drone& drone, const kdl::Tree& tree)
  : super(drone, tree), fk_solver_(tree), inertia_solver_(tree), z_rotors_(drone, Z_POSITIVE)
{
}

bool MultiRotorMixer_pinv::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures())
    return false;

  if (!fk_solver_.updateInternalDataStructures())
    return false;
  if (!inertia_solver_.updateInternalDataStructures())
    return false;
  if (!z_rotors_.updateInternalDataStructures())
    return false;

  E_.conservativeResize(NoChange, z_rotors_.count());
  E_.bottomRows<1>().setOnes();  // 推力和等式の左辺

  x_.conservativeResize(z_rotors_.count());

  return true;
}

bool MultiRotorMixer_pinv::solve(
  const kdl::JntArray& cur_q,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_dgyro_B,
  const double& tar_thrusts_sum,
  const kdl::Vector& ext_torque_B)
{
  assert(tar_thrusts_sum > 0);

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

  // EoM行列等式の左辺
  for (size_t i = 0; i < z_rotors_.count(); ++i)
  {
    const auto& rotor = z_rotors_.rotor(i);

    if (rotor_alive_.at(rotor->link_name))
    {
      const auto& B_Pos_B2P = fk_solver_.getFrame(rotor->link_name).p;

      const auto elem = tree_.getSegment(rotor->link_name)->second;
      const auto& B_Rot_Par = fk_solver_.getFrame(elem.parent->first).M;
      const auto axis_B = B_Rot_Par * elem.segment.joint().axis();

      const auto d = rotor->sign();
      const auto& cm = rotor->moment_const;
      const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
      E_.block<3, 1>(0, i) = (B_Pos_G2P * axis_B - (d * cm) * axis_B).data;
    }
    else
    {
      // ロータが死んでいる時は推力から期待の運動への伝達をゼロにすることで最適推力がゼロになるよう仕向ける
      E_.block<3, 1>(0, i).setZero();
    }
  }

  // EoM行列等式の右辺
  f_.head<3>() = (I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B) - ext_torque_B).data;  // [Nm]

  // 推力和等式の右辺
  f_(3) = tar_thrusts_sum;

  // Ex = f を解く
  // TODO: Rank(E) < 4 (方程式が解けない) の場合に行ごとに優先度 (atti > thrust > yaw) をつける
  x_ = E_.jacobiSvd(ComputeThinU | ComputeThinV).solve(f_);

  return true;
}

const VectorXd& MultiRotorMixer_pinv::getThrusts() const
{
  return x_;
}
}  // namespace tobas
