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
WindEstimator::WindEstimator(ros::NodeHandle nh, ros::NodeHandle pnh, const string& name)
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

Matrix3d WindEstimator::velCoef(const Euler& R_W_B)
{
  double sum = 0.;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    const auto& rotor_idx = z_rotors_.rotorIdx(i);
    const auto& cd = z_rotors_.dragConstant(i);
    const auto& rot_speed = rotor_speeds_->speeds[rotor_idx];
    sum += cd * abs(rot_speed);
  }

  const Matrix3d R_B_W = R_W_B.toRotation().Inverse().data;
  return (sum / mass_) * E_XY * R_B_W;
}

void WindEstimator::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void WindEstimator::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  if (rotor_speeds_ == nullptr)
    return;

  const Matrix3d R_W_B = pt->pose.euler.toRotation().data;
  const Vector3d vel_W = R_W_B * pt->twist.vel.data;
  const Vector3d& acc_B = pt->accel.linear.data;
  const Vector3d grav_B = R_W_B.transpose() * GRAV_W;
  const Matrix3d Cv = velCoef(pt->pose.euler);

  const Vector2d right = (acc_B + grav_B + Cv * vel_W).head(2);
  const Vector3d wind_W = Cv.colPivHouseholderQr().solve(right);

  // Publish wind message
  auto wind_msg = boost::make_shared<tobas_msgs::Wind>();
  wind_msg->header.frame_id = tobas::kWorldFrame;
  wind_msg->header.stamp = pt->header.stamp;
  wind_msg->vel.data = wind_W;
  wind_pub_.publish(wind_msg);
}

void WindEstimator::rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds)
{
  rotor_speeds_ = rotor_speeds;
}
}  // namespace tobas_mr_wind_estimation
