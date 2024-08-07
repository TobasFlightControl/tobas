#pragma once

#include "./mr_dynamics.hpp"

namespace tobas
{
struct AccelAttitudeConverterConfig
{
  double max_attitude;
  double h_force_comp_rate;
};

class AccelAttitudeConverter
{
public:
  explicit AccelAttitudeConverter(const Drone& drone, const kdl::Tree& tree);

  void updateInternalDataStructures();

  /* 空気効力を考慮する場合． */
  void update(
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_vel_B,
    const kdl::Vector& cur_wind_W,
    const std::vector<double>& cur_rotor_speeds,
    const kdl::Vector& tar_acc_W,
    double& thrust_out,
    double& roll_out,
    double& pitch_out);

  /* 空気効力を考慮しない場合． */
  void update(
    const kdl::Rotation& cur_rot,
    const kdl::Vector& tar_acc_W,
    double& thrust_out,
    double& roll_out,
    double& pitch_out);

  void configure(const AccelAttitudeConverterConfig& cfg);

private:
  AccelAttitudeConverterConfig cfg_;

  const Drone& drone_;
  const kdl::Tree& tree_;

  tobas::MultirotorDynamicsComponents dynamics_;

  const kdl::Vector grav_W_;
  const kdl::Vector zero_;

  double roll_, pitch_, yaw_;
};
}  // namespace tobas
