#pragma once

#include <eigen3/Eigen/Core>

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
double motorConstFromThrustStand(const Eigen::VectorXd& rpms, const Eigen::VectorXd& thrusts);
double momentConstFromThrustStand(const Eigen::VectorXd& thrusts, const Eigen::VectorXd& torques);

double motorConstFromUiuc(const Eigen::VectorXd& cts, double d);
double momentConstFromUiuc(const Eigen::VectorXd& cts, const Eigen::VectorXd& cps, double d);
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
