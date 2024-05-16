#pragma once

#include <tobas_tools/drone.hpp>
#include <tobas_mr_common/dynamics.hpp>

namespace tobas_mr_common
{
struct AccelAttitudeConverterConfig
{
  double max_attitude;
  double h_force_comp_rate;
};

class AccelAttitudeConverter
{
public:
  explicit AccelAttitudeConverter(const tobas::Drone& drone);

  void updateInternalDataStructures();

  /* 空気効力を考慮する場合． */
  void update(
    const tobas_kdl::Rotation& cur_rot,
    const tobas_kdl::Vector& cur_vel_B,
    const tobas_kdl::Vector& cur_wind_W,
    const std::vector<double>& cur_rotor_speeds,
    const tobas_kdl::Vector& tar_acc_W,
    double& thrust_out,
    double& roll_out,
    double& pitch_out);

  /* 空気効力を考慮しない場合． */
  void update(
    const tobas_kdl::Rotation& cur_rot,
    const tobas_kdl::Vector& tar_acc_W,
    double& thrust_out,
    double& roll_out,
    double& pitch_out);

  void configure(const AccelAttitudeConverterConfig& cfg);

private:
  AccelAttitudeConverterConfig cfg_;

  const tobas::Drone& drone_;
  tobas_mr_common::MultirotorDynamicsComponents dynamics_;

  const tobas_kdl::Vector grav_W_;
  const tobas_kdl::Vector zero_;

  double roll_, pitch_, yaw_;
};
}  // namespace tobas_mr_common
