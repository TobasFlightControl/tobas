#pragma once

#include <chrono>

#include <tobas_kdl/frame_acc.hpp>
#include <tobas_kdl/trajectory.hpp>
#include <tobas_kdl/tree_fk_solver_pos.hpp>

namespace lr_tools
{
/* デカルト座標系における遊脚の足先の状態を計算する． */
class SwingLegController
{
  static constexpr double kDefaultRaibertGain = 0.03;
  static constexpr double kDefaultStandPeriod = 1.;
  static constexpr double kDefaultSwingPeriod = 1.;
  static constexpr double kDefaultClearance = 0.01;

  using TimeType = std::chrono::steady_clock::time_point;
  using DurationType = std::chrono::duration<double>;

public:
  explicit SwingLegController(
    const kdl::Tree& tree,
    const std::vector<std::string>& thigh_names,
    const std::vector<std::string>& foot_names);

  bool updateInternalDataStructures();

  void reset();

  bool update(
    double z,
    const kdl::Vector& G_Vel_GB,
    const kdl::Rotation& W_Rot_B,
    const kdl::Vector& G_Gyro_GB,
    const kdl::JntArray& q,
    const std::vector<bool>& is_stand,
    const TimeType& cur_time);

  bool setRaibertGain(double raibert_gain);
  bool setClearance(double clearance);
  bool setGaitParams(double stand_period, double swing_period);
  bool setVelocity(double vx, double vy, double yawrate);

  inline const kdl::VectorAcc& getFootState(size_t leg) const;

private:
  const kdl::Tree& tree_;
  const std::vector<std::string> thigh_names_, foot_names_;
  const size_t nc_;  // The number of contact points

  // Config
  double raibert_gain_ = kDefaultRaibertGain;  // [-]
  double clearance_ = kDefaultClearance;       // [m]
  double stand_period_ = kDefaultStandPeriod;  // [s]
  double swing_period_ = kDefaultSwingPeriod;  // [s]
  double vx_ = 0.;                             // [m/s]
  double vy_ = 0.;                             // [m/s]
  double yawrate_ = 0.;                        // [rad/s]

  kdl::TreeFkSolverPos fk_solver_;

  std::vector<kdl::CycloidGenerator3d> ref_traj_;  // {gnd}から見た{gnd}に対する{foot}の軌道を生成する
  std::vector<bool> is_stand_prev_;                // 各足の接地状態
  std::vector<TimeType> t_switch_;                 // 立脚から遊脚に切り替わった時刻
  std::vector<kdl::VectorAcc> B_Tdd_BF_;           // {base}から見た{base}に対する{foot}の状態
  std::vector<kdl::Vector> thigh_0_;               // {base}から見た{base}に対する足の根本の位置
  kdl::VectorAcc G_Tdd_GF_;                        // {gnd}から見た{gnd}に対する{foot}の状態
  double roll_, pitch_, yaw_;                      // W_Rot_B

  void setThighOrigins();
};

inline const kdl::VectorAcc& SwingLegController::getFootState(size_t leg) const
{
  return B_Tdd_BF_[leg];
}
}  // namespace lr_tools
