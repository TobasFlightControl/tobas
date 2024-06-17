#pragma once

#include <tobas_linear_control/kalman_filter.hpp>
#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>

namespace tobas_legged_tools
{
/**
 * @brief Contact estimator.
 * Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains [Bledt+, 2018]
 */
class ContactEstimator
{
  static constexpr size_t kNumMeasurements = 2;
  static constexpr double kContactProbThreshold = 0.5;  // [-]
  static constexpr double kMeanFootHeight = 0.;         // [m]

  // TODO: ユーザが調整できるようにする
  static constexpr double kPredictionNoiseVariance = 0.998;    // [-]
  static constexpr double kFootHeightNoiseVariance = 0.841;    // [-]
  static constexpr double kContactForceNoiseVariance = 0.930;  // [-]

  static constexpr double kDefaultPredictionErfVariance = 0.05;   // [-]
  static constexpr double kDefaultFootHeightErfVariance = 0.1;    // [m^2]
  static constexpr double kDefaultContactForceErfVariance = 25.;  // [N^2]

public:
  enum state_t
  {
    Contact,
    Swing,
    Early,
    Late,
  };

  explicit ContactEstimator(const tobas_kdl::Tree& tree, const std::vector<std::string>& foot_names);

  void updateInternalDataStructures();

  void update(
    const tobas_kdl::Frame& T,
    const tobas_kdl::JntArray& q,
    const std::vector<double>& contact_forces,
    const std::vector<bool>& cpg_states,
    const std::vector<double>& cpg_subphases);
  void reset();

  void setPredictionVariance(const double& var);
  void setFootHeightVariance(const double& var);
  void setContactForceVariance(const double& var);

  bool isContact(const size_t& idx) const;
  double contactProbability(const size_t& idx) const;
  double contactProbabilityFromCPG(const size_t& idx) const;

private:
  const tobas_kdl::Tree& tree_;
  tobas_kdl::TreeFkSolverPos fk_solver_;
  tobas_kdl::TreeJntToInertiaSolver inertia_solver_;

  const std::vector<std::string> foot_names_;
  const size_t nc_;              // The number of contact points
  std::vector<state_t> states_;  // FSMの状態
  ctrl::KalmanFilter kf_;

  double mean_force_;
  double erfden_pred_, erfden_height_, erfden_force_;

  void setupKalmanFilter();

  Eigen::VectorXd calcProbs_height(const tobas_kdl::Frame& T, const tobas_kdl::JntArray& q);
  Eigen::VectorXd calcProbs_force(const std::vector<double>& contact_forces);
  Eigen::VectorXd calcProbs_pred(const std::vector<bool>& cpg_states, const std::vector<double>& cpg_subphases);
};
}  // namespace tobas_legged_tools
