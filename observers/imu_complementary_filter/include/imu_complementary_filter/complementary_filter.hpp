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

  Eigen::Vector3d getAngularVelocityBias() const;

  void setOrientation(const Eigen::Quaterniond& q_WB);

  /* {world} -> {base} */
  Eigen::Quaterniond getOrientation() const;

  void setReferenceMagneticField(double ref_mag_north, double ref_mag_east, double ref_mag_down);

  /**
   * @brief Update from accelerometer, gyroscope, and magnetometer data.
   *
   * @param a gravity vector, units irrelevant
   * @param w Angular veloctiy [rad/s]
   * @param m Magnetic field, units irrelevant
   * @param dt time delta [s]
   */
  void
  update(const Eigen::Vector3d& a, const Eigen::Vector3d& w, const Eigen::Vector3d& m, double dt);

  /* Reset the filter to the initial state. */
  void reset();

private:
  double gravity_;
  double gain_acc_;    // Accel gain parameter for the complementary filter, belongs in [0, 1].
  double gain_mag_;    // Magnetic gain parameter for the complementary filter, belongs in [0, 1].
  double bias_alpha_;  // Bias estimation gain parameter, belongs in [0, 1].
  bool do_bias_estimation_;  // Parameter whether to do bias estimation or not.
  bool do_adaptive_gain_;    // Parameter whether to do adaptive gain or not.

  bool is_initialized_;

  Eigen::Quaterniond q_WF_;  // {world} -> {fixed}
  Eigen::Quaterniond q_BF_;  // {base} -> {fixed}
  Eigen::Vector3d w_prev_;   // Previous angular velocity
  Eigen::Vector3d w_bias_;   // Bias in angular velocity

  void updateBiases(const Eigen::Vector3d& a, const Eigen::Vector3d& w);
  bool checkState(const Eigen::Vector3d& a, const Eigen::Vector3d& w) const;
  Eigen::Quaterniond getPrediction(const Eigen::Vector3d& w, double dt) const;
  Eigen::Quaterniond getMeasurement(const Eigen::Vector3d& a, const Eigen::Vector3d& m) const;
  Eigen::Quaterniond getAccCorrection(const Eigen::Vector3d& a, const Eigen::Quaterniond& p) const;
  Eigen::Quaterniond getMagCorrection(const Eigen::Vector3d& m, const Eigen::Quaterniond& p) const;
  double getAdaptiveGain(double alpha, const Eigen::Vector3d& a) const;
};
