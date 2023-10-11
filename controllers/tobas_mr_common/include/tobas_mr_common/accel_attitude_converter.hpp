#pragma once

#include <dh_kdl/frames.hpp>

namespace tobas_mr_common
{
struct AccelAttitudeConverterConfig
{
  double max_hor_acc;
  double max_ver_acc;
  double max_attitude;
};

class AccelAttitudeConverter
{
public:
  explicit AccelAttitudeConverter();

  void update(
    const KDL::Vector& tar_acc,
    const double& yaw,
    double& U_out,
    double& roll_out,
    double& pitch_out);
  void configure(const AccelAttitudeConverterConfig& cfg);

private:
  double mass_;

  AccelAttitudeConverterConfig cfg_;
};
}  // namespace tobas_mr_common
