#include "../../include/orientation_estimation_complement/orientation_estimator.hpp"
#include "../../include/orientation_estimation_complement/utils.hpp"

// Constants
#define ACC_THRESHOLD 0.1
#define ANGVEL_THRESHOLD 0.2
#define DELTA_ANGVEL_THRESHOLD 0.01

// Default parameters
#define DEFAULT_GRAVITY 9.80665
#define DEFAULT_GAIN_ACC 0.01
#define DEFAULT_GAIN_MAG 0.01
#define DEFAULT_BIAS_ALPHA 0.01
#define DEFAULT_DO_BIAS_ESTIMATION true
#define DEFAULT_DO_ADAPTIVE_GAIN false

#define SQR(x) (x * x)

using namespace std;
using namespace Eigen;

OrientationEstimator::OrientationEstimator()
  : gravity_(DEFAULT_GRAVITY),
    gain_acc_(DEFAULT_GAIN_ACC),
    gain_mag_(DEFAULT_GAIN_MAG),
    bias_alpha_(DEFAULT_BIAS_ALPHA),
    do_bias_estimation_(DEFAULT_DO_BIAS_ESTIMATION),
    do_adaptive_gain_(DEFAULT_DO_ADAPTIVE_GAIN),
    is_initialized_(false),
    q_WF_(Quaterniond::Identity()),
    q_BF_(Quaterniond::Identity()),
    w_prev_(Vector3d::Zero()),
    w_bias_(Vector3d::Zero())
{
}

bool OrientationEstimator::setGravity(double gravity)
{
  if (gravity >= 0.)
  {
    gravity_ = gravity;
    return true;
  }
  else
  {
    return false;
  }
}

bool OrientationEstimator::setGainAcc(double gain)
{
  if (0. <= gain && gain <= 1.)
  {
    gain_acc_ = gain;
    return true;
  }
  else
  {
    return false;
  }
}
bool OrientationEstimator::setGainMag(double gain)
{
  if (0. <= gain && gain <= 1.)
  {
    gain_mag_ = gain;
    return true;
  }
  else
  {
    return false;
  }
}

bool OrientationEstimator::setBiasAlpha(double bias_alpha)
{
  if (0. <= bias_alpha && bias_alpha <= 1.)
  {
    bias_alpha_ = bias_alpha;
    return true;
  }
  else
  {
    return false;
  }
}

void OrientationEstimator::setDoBiasEstimation(bool do_bias_estimation)
{
  do_bias_estimation_ = do_bias_estimation;
}

void OrientationEstimator::setDoAdaptiveGain(bool do_adaptive_gain)
{
  do_adaptive_gain_ = do_adaptive_gain;
}

Vector3d OrientationEstimator::getAngularVelocityBias() const
{
  return w_bias_;
}

void OrientationEstimator::setOrientation(const Quaterniond& q_WB)
{
  q_BF_ = (q_WB.conjugate() * q_WF_).normalized();
}

Quaterniond OrientationEstimator::getOrientation() const
{
  return (q_WF_ * q_BF_.conjugate()).normalized();
}

void OrientationEstimator::setReferenceMagneticField(double ref_mag_north, double ref_mag_east)
{
  double yaw_angle = -atan2(ref_mag_east, ref_mag_north);
  q_WF_.w() = cos(yaw_angle / 2.);
  q_WF_.x() = 0.;
  q_WF_.y() = 0.;
  q_WF_.z() = sin(yaw_angle / 2.);
}

void OrientationEstimator::update(
  const Vector3d& a,
  const Vector3d& w,
  const Vector3d& m,
  double dt)
{
  if (!is_initialized_)
  {
    // First time - ignore prediction
    q_BF_ = getMeasurement(a, m);
    is_initialized_ = true;
    return;
  }

  // Bias estimation
  if (do_bias_estimation_)
  {
    updateBiases(a, w);
  }

  // Prediction
  const Quaterniond q_pred = getPrediction(w, dt);

  // Correction (from acc):
  // q_tmp = q_pred * [(1-gain) * qI + gain * dq_acc]
  // where qI = identity quaternion
  Quaterniond dq_acc = getAccCorrection(a, q_pred);
  const double alpha = do_adaptive_gain_ ? getAdaptiveGain(gain_acc_, a) : gain_acc_;
  scaleQuaternion(alpha, dq_acc);

  const Quaterniond q_tmp = (q_pred * dq_acc).normalized();

  // Correction (from mag):
  // q_ = q_tmp * [(1-gain) * qI + gain * dq_mag]
  // where qI = identity quaternion
  Quaterniond dq_mag = getMagCorrection(m, q_tmp);
  scaleQuaternion(gain_mag_, dq_mag);

  q_BF_ = q_tmp * dq_mag;
  q_BF_.normalize();
}

void OrientationEstimator::reset()
{
  is_initialized_ = false;
  q_BF_ = Quaterniond::Identity();
  w_prev_ = Vector3d::Zero();
  w_bias_ = Vector3d::Zero();
}

void OrientationEstimator::updateBiases(const Vector3d& a, const Vector3d& w)
{
  if (checkState(a, w))
  {
    w_bias_ += bias_alpha_ * (w - w_bias_);
  }
  w_prev_ = w;
}

bool OrientationEstimator::checkState(const Vector3d& a, const Vector3d& w) const
{
  if (abs(a.norm() - gravity_) > ACC_THRESHOLD)
  {
    return false;
  }

  if ((w - w_bias_).cwiseAbs().maxCoeff() > ANGVEL_THRESHOLD)
  {
    return false;
  }

  if ((w - w_prev_).cwiseAbs().maxCoeff() > DELTA_ANGVEL_THRESHOLD)
  {
    return false;
  }

  return true;
}

Quaterniond OrientationEstimator::getPrediction(const Vector3d& w, double dt) const
{
  const Vector3d w_unb = w - w_bias_;

  Quaterniond q_pred = q_BF_;
  q_pred.w() += 0.5 * dt * (w_unb.x() * q_BF_.x() + w_unb.y() * q_BF_.y() + w_unb.z() * q_BF_.z());
  q_pred.x() += 0.5 * dt * (-w_unb.x() * q_BF_.w() - w_unb.y() * q_BF_.z() + w_unb.z() * q_BF_.y());
  q_pred.y() += 0.5 * dt * (w_unb.x() * q_BF_.z() - w_unb.y() * q_BF_.w() - w_unb.z() * q_BF_.x());
  q_pred.z() += 0.5 * dt * (-w_unb.x() * q_BF_.y() + w_unb.y() * q_BF_.x() - w_unb.z() * q_BF_.w());

  return q_pred.normalized();
}

Quaterniond OrientationEstimator::getMeasurement(const Vector3d& a, const Vector3d& m) const
{
  // q_acc is the quaternion obtained from the acceleration vector
  // representing the orientation of the Global frame wrt the Local frame with
  // arbitrary yaw (intermediary frame). q3_acc is defined as 0.
  Quaterniond q_acc;
  const Vector3d a_norm = a.normalized();
  if (a_norm.z() >= 0.)
  {
    q_acc.w() = sqrt((a_norm.z() + 1.) / 2.);
    q_acc.x() = -a_norm.y() / (2. * q_acc.w());
    q_acc.y() = a_norm.x() / (2. * q_acc.w());
    q_acc.z() = 0.;
  }
  else
  {
    double X = sqrt((1. - a_norm.z()) / 2.);
    q_acc.w() = -a_norm.y() / (2. * X);
    q_acc.x() = X;
    q_acc.y() = 0.;
    q_acc.z() = a_norm.x() / (2. * X);
  }

  // l is the magnetic field reading,
  // rotated into the intermediary frame by the inverse of q_acc.
  // l = R(q_acc)^-1 m
  const Vector3d l = q_acc.conjugate() * m;

  // q_mag is the quaternion that rotates the Global frame (North West Up)
  // into the intermediary frame. q1_mag and q2_mag are defined as 0.
  Quaterniond q_mag;
  const double gamma = SQR(l.x()) + SQR(l.y());
  const double beta = sqrt(gamma + l.x() * sqrt(gamma));
  q_mag.w() = beta / (sqrt(2. * gamma));
  q_mag.x() = 0.;
  q_mag.y() = 0.;
  q_mag.z() = l.y() / (sqrt(2.) * beta);

  // The quaternion multiplication between q_acc and q_mag represents the
  // quaternion, orientation of the Global frame wrt the local frame.
  // q = q_acc times q_mag
  return (q_acc * q_mag).normalized();
}

Quaterniond OrientationEstimator::getAccCorrection(const Vector3d& a, const Quaterniond& p) const
{
  // Normalize acceleration vector
  const Vector3d a_norm = a.normalized();

  // Acceleration reading rotated into the fixed frame by the inverse
  // predicted quaternion (predicted gravity):
  const Vector3d g = p.conjugate() * a_norm;

  // Delta quaternion that rotates the predicted gravity into the real gravity:
  Quaterniond dq;
  dq.w() = sqrt((g.z() + 1.) / 2.);
  dq.x() = -g.y() / (2. * dq.w());
  dq.y() = g.x() / (2. * dq.w());
  dq.z() = 0.;

  return dq;
}

Quaterniond OrientationEstimator::getMagCorrection(const Vector3d& m, const Quaterniond& p) const
{
  // Magnetic reading rotated into the world frame by the inverse predicted quaternion:
  const Vector3d l = p.conjugate() * m;

  // Delta quaternion that rotates the l so that it lies in the xz-plane (points north):
  Quaterniond dq;
  const double gamma = SQR(l.x()) + SQR(l.y());
  const double beta = sqrt(gamma + l.x() * sqrt(gamma));
  dq.w() = beta / (sqrt(2. * gamma));
  dq.x() = 0.;
  dq.y() = 0.;
  dq.z() = l.y() / (sqrt(2.) * beta);

  return dq;
}

double OrientationEstimator::getAdaptiveGain(double alpha, const Vector3d& a) const
{
  constexpr double error1 = 0.1;
  constexpr double error2 = 0.2;
  constexpr double m = 1. / (error1 - error2);
  constexpr double b = 1. - m * error1;

  const double error = fabs(a.norm() - gravity_) / gravity_;
  double factor;

  if (error < error1)
  {
    factor = 1.;
  }
  else if (error < error2)
  {
    factor = m * error + b;
  }
  else
  {
    factor = 0.;
  }

  return factor * alpha;
}
