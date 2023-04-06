#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

class ComplementaryFilter
{
public:
  ComplementaryFilter();

  bool setGravity(double gravity);
  bool setGainAcc(double gain);
  bool setGainMag(double gain);
  bool setBiasAlpha(double bias_alpha);
  void setDoBiasEstimation(bool do_bias_estimation);
  void setDoAdaptiveGain(bool do_adaptive_gain);

  double getAngularVelocityBiasX() const;
  double getAngularVelocityBiasY() const;
  double getAngularVelocityBiasZ() const;

  // Set the orientation, as a Hamilton Quaternion, of the body frame wrt the world frame.
  void setOrientation(double qWB_0, double qWB_1, double qWB_2, double qWB_3);

  // Get the orientation, as a Hamilton Quaternion, of the body frame wrt the world frame.
  void getOrientation(double& qWB_0, double& qWB_1, double& qWB_2, double& qWB_3) const;

  void setReferenceMagneticField(double ref_mag_north, double ref_mag_east, double ref_mag_down);

  // Update from accelerometer and gyroscope data.
  // [ax, ay, az]: Normalized gravity vector.
  // [wx, wy, wz]: Angular veloctiy, in rad / s.
  // dt: time delta, in seconds.
  void update(double ax, double ay, double az, double wx, double wy, double wz, double dt);

  // Update from accelerometer, gyroscope, and magnetometer data.
  // [ax, ay, az]: Normalized gravity vector.
  // [wx, wy, wz]: Angular veloctiy, in rad / s.
  // [mx, my, mz]: Magnetic field, units irrelevant.
  // dt: time delta, in seconds.
  void update(
    double ax,
    double ay,
    double az,
    double wx,
    double wy,
    double wz,
    double mx,
    double my,
    double mz,
    double dt);

  // Reset the filter to the initial state.
  void reset();

private:
  double gravity_;
  double gain_acc_;    // Accel gain parameter for the complementary filter, belongs in [0, 1].
  double gain_mag_;    // Magnetic gain parameter for the complementary filter, belongs in [0, 1].
  double bias_alpha_;  // Bias estimation gain parameter, belongs in [0, 1].
  bool do_bias_estimation_;  // Parameter whether to do bias estimation or not.
  bool do_adaptive_gain_;    // Parameter whether to do adaptive gain or not.

  bool is_initialized_;
  bool is_steady_state_;

  // The orientation as a Hamilton quaternion (q0 is the scalar). Represents
  // the orientation of the fixed frame wrt the body frame.
  double qBF_0_, qBF_1_, qBF_2_, qBF_3_;

  // the orientation of the fixed frame wrt the world frame.
  double qWF_0_, qWF_1_, qWF_2_, qWF_3_;

  // Bias in angular velocities;
  double wx_prev_, wy_prev_, wz_prev_;

  // Bias in angular velocities;
  double wx_bias_, wy_bias_, wz_bias_;

  void updateBiases(double ax, double ay, double az, double wx, double wy, double wz);

  bool checkState(double ax, double ay, double az, double wx, double wy, double wz) const;

  void getPrediction(
    double wx,
    double wy,
    double wz,
    double dt,
    double& q0_pred,
    double& q1_pred,
    double& q2_pred,
    double& q3_pred) const;

  void getMeasurement(
    double ax,
    double ay,
    double az,
    double& q0_meas,
    double& q1_meas,
    double& q2_meas,
    double& q3_meas);

  void getMeasurement(
    double ax,
    double ay,
    double az,
    double mx,
    double my,
    double mz,
    double& q0_meas,
    double& q1_meas,
    double& q2_meas,
    double& q3_meas);

  void getAccCorrection(
    double ax,
    double ay,
    double az,
    double p0,
    double p1,
    double p2,
    double p3,
    double& dq0,
    double& dq1,
    double& dq2,
    double& dq3);

  void getMagCorrection(
    double mx,
    double my,
    double mz,
    double p0,
    double p1,
    double p2,
    double p3,
    double& dq0,
    double& dq1,
    double& dq2,
    double& dq3);

  double getAdaptiveGain(double alpha, double ax, double ay, double az);
};

// Utility math functions:

void normalizeVector(double& x, double& y, double& z);

void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3);

void scaleQuaternion(double gain, double& dq0, double& dq1, double& dq2, double& dq3);

void invertQuaternion(
  double q0,
  double q1,
  double q2,
  double q3,
  double& q0_inv,
  double& q1_inv,
  double& q2_inv,
  double& q3_inv);

void quaternionMultiplication(
  double p0,
  double p1,
  double p2,
  double p3,
  double q0,
  double q1,
  double q2,
  double q3,
  double& r0,
  double& r1,
  double& r2,
  double& r3);

void rotateVectorByQuaternion(
  double x,
  double y,
  double z,
  double q0,
  double q1,
  double q2,
  double q3,
  double& vx,
  double& vy,
  double& vz);
