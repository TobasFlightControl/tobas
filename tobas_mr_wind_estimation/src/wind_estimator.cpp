#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Wind.h>

#include "../include/tobas_mr_wind_estimation/wind_estimator.hpp"

#define E_XY DiagonalMatrix<double, 3>(1, 1, 0)
#define GRAV_W Vector3d(0, 0, tobas::kGravity)

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_wind_estimation
{
WindEstimator::WindEstimator(
  ros::NodeHandle nh,
  ros::NodeHandle pnh,
  string name = ros::this_node::getName())
  : super(nh, pnh, name),
    fk_solver_(drone_.tree()),
    inertia_solver_(drone_.tree()),
    z_rotors_(drone_, tobas::Axis::Z_POSITIVE)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  updateInternalDataStructures();

  registerPublishers();
  registerSubscribers();
}

void WindEstimator::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();

  mass_ = inertia_solver_.JntToMass();
}

void WindEstimator::getRosParams()
{
}

void WindEstimator::registerPublishers()
{
  wind_pub_ = nh_.advertise<tobas_msgs::Wind>(tobas::kWindTopic, 1);
}

void WindEstimator::registerSubscribers()
{
  pt_sub_ =
    nh_.subscribe(tobas::kPoseTwistTopic, 1, &WindEstimator::poseTwistCb, this, tcpNoDelay());
  rotor_speeds_sub_ =
    nh_.subscribe(tobas::kRotorSpeedsTopic, 1, &WindEstimator::rotorSpeedsCb, this, tcpNoDelay());
}

Eigen::Matrix3d WindEstimator::velCoef(const KDL::Euler& R_W_B)
{
  // TODO
}

void WindEstimator::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  if (rotor_speeds_ == nullptr)
    return;

  const auto acc_meas = pt->accel.linear * pt->pose.euler.Inverse(GRAV_W);
  const Matrix3d vel_coef = velCoef(pt->pose.euler);
}

void WindEstimator::rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds)
{
  rotor_speeds_ = rotor_speeds;
}
}  // namespace tobas_mr_wind_estimation
