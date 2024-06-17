#pragma once

#include <boost/array.hpp>

#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_linear_control/kalman_filter.hpp>
#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>

namespace tobas_legged_tools
{
/**
 * @brief Contact estimator.
 * Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains [Bledt+, 2018]
 */
template <size_t N>
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

  explicit ContactEstimator(const tobas_kdl::Tree& tree, const boost::array<std::string, N>& foot_names);

  void updateInternalDataStructures();

  void update(
    const tobas_kdl::Frame& T,
    const tobas_kdl::JntArray& q,
    const boost::array<double, N>& contact_forces,
    const boost::array<bool, N>& cpg_states,
    const boost::array<double, N>& cpg_subphases);
  void reset();

  void setPredictionVariance(const double& var);
  void setFootHeightVariance(const double& var);
  void setContactForceVariance(const double& var);

  bool isContact(const size_t& idx) const;
  double contactProbability(const size_t& idx) const;
  double contactProbabilityFromCPG(const size_t& idx) const;

private:
  const tobas_kdl::Tree tree_;
  tobas_kdl::TreeFkSolverPos fk_solver_;
  tobas_kdl::TreeJntToInertiaSolver inertia_solver_;

  boost::array<std::string, N> foot_names_;

  ctrl::KalmanFilter kf_;
  boost::array<state_t, N> states_;  // FSMの状態

  double mean_force_;
  double erfden_pred_, erfden_height_, erfden_force_;

  void setupKalmanFilter();

  Eigen::Matrix<double, N, 1> calcProbs_height(const tobas_kdl::Frame& T, const tobas_kdl::JntArray& q);
  Eigen::Matrix<double, N, 1> calcProbs_force(const boost::array<double, N>& contact_forces);
  Eigen::Matrix<double, N, 1>
  calcProbs_pred(const boost::array<bool, N>& cpg_states, const boost::array<double, N>& cpg_subphases);
};

template <size_t N>
ContactEstimator<N>::ContactEstimator(const tobas_kdl::Tree& tree, const boost::array<std::string, N>& foot_names)
  : tree_(tree), fk_solver_(tree), inertia_solver_(tree), foot_names_(foot_names), kf_(N, N, N * kNumMeasurements, N)
{
  setPredictionVariance(kDefaultPredictionErfVariance);
  setFootHeightVariance(kDefaultFootHeightErfVariance);
  setContactForceVariance(kDefaultContactForceErfVariance);

  setupKalmanFilter();
  updateInternalDataStructures();
  reset();
}

template <size_t N>
void ContactEstimator<N>::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  inertia_solver_.JntToCart(tobas_kdl::JntArray::Zero(tree_.getNrOfJoints()));
  const auto& mass = inertia_solver_.getInertia().getMass();
  mean_force_ = mass * tobas_std::kGravity / N;
}

template <size_t N>
void ContactEstimator<N>::update(
  const tobas_kdl::Frame& T,
  const tobas_kdl::JntArray& q,
  const boost::array<double, N>& contact_forces,
  const boost::array<bool, N>& cpg_states,
  const boost::array<double, N>& cpg_subphases)
{
  // カルマンフィルタで接触確率を推定
  kf_.y.segment<N>(0) = calcProbs_height(T, q);
  kf_.y.segment<N>(N) = calcProbs_force(contact_forces);  // TODO: 論文通り接触センサに加えトルクも利用
  kf_.u = calcProbs_pred(cpg_states, cpg_subphases);
  kf_.update();

  // 接触状態を更新 (Fig.10)
  for (size_t l = 0; l < N; ++l)
  {
    const auto contact_detected = (kf_.state()(l) > kContactProbThreshold);  // 接触検知されたか否か

    switch (states_[l])
    {
      case Contact:
      {
        if (!cpg_states[l])
          states_[l] = Swing;
        break;
      }
      case Swing:
      {
        if (cpg_states[l] && contact_detected)
          states_[l] = Contact;
        else if (cpg_states[l])
          states_[l] = Late;
        else if (contact_detected)
          states_[l] = Early;
        break;
      }
      case Early:
      {
        if (cpg_states[l])
          states_[l] = Contact;
        break;
      }
      case Late:
      {
        if (contact_detected)
          states_[l] = Contact;
        break;
      }
      default:
      {
        throw std::runtime_error("Invalid contact state");
      }
    }
  }
}

template <size_t N>
void ContactEstimator<N>::reset()
{
  const auto ones = Eigen::VectorXd::Ones(N);
  kf_.initialize(ones, ones.asDiagonal());

  states_.fill(Contact);
}

template <size_t N>
void ContactEstimator<N>::setPredictionVariance(const double& var)
{
  erfden_pred_ = sqrt(2 * var);
}

template <size_t N>
void ContactEstimator<N>::setFootHeightVariance(const double& var)
{
  erfden_height_ = sqrt(2 * var);
}

template <size_t N>
void ContactEstimator<N>::setContactForceVariance(const double& var)
{
  erfden_force_ = sqrt(2 * var);
}

template <size_t N>
bool ContactEstimator<N>::isContact(const size_t& idx) const
{
  return states_[idx] == Contact || states_[idx] == Early;
}

template <size_t N>
double ContactEstimator<N>::contactProbability(const size_t& idx) const
{
  return kf_.state()(idx);
}

template <size_t N>
double ContactEstimator<N>::contactProbabilityFromCPG(const size_t& idx) const
{
  return kf_.u(idx);
}

template <size_t N>
void ContactEstimator<N>::setupKalmanFilter()
{
  kf_.ss.A.setZero();
  kf_.ss.B.setIdentity();
  kf_.ss.C.block<N, N>(0, 0).setIdentity();
  kf_.ss.C.block<N, N>(N, 0).setIdentity();
  kf_.Bv.setIdentity();

  kf_.Q.setZero();
  kf_.Q.diagonal().fill(kPredictionNoiseVariance);

  kf_.R.setZero();
  kf_.R.diagonal().segment<N>(0).fill(kFootHeightNoiseVariance);
  kf_.R.diagonal().segment<N>(N).fill(kContactForceNoiseVariance);
}

template <size_t N>
Eigen::Matrix<double, N, 1>
ContactEstimator<N>::calcProbs_height(const tobas_kdl::Frame& T, const tobas_kdl::JntArray& q)
{
  Eigen::Matrix<double, N, 1> res;
  for (size_t l = 0; l < N; ++l)
  {
    fk_solver_.JntToCart(q, foot_names_[l]);
    const auto F_Pos_FC = T.M * fk_solver_.getFrame().p;
    const auto height = T.p.z() + F_Pos_FC.z();
    res(l) = 0.5 * (1 + erf((kMeanFootHeight - height) / erfden_height_));
  }
  return res;
}

template <size_t N>
Eigen::Matrix<double, N, 1> ContactEstimator<N>::calcProbs_force(const boost::array<double, N>& contact_forces)
{
  Eigen::Matrix<double, N, 1> res;
  for (size_t l = 0; l < N; ++l)
    res(l) = 0.5 * (1 + erf((contact_forces[l] - mean_force_) / erfden_force_));
  return res;
}

template <size_t N>
Eigen::Matrix<double, N, 1> ContactEstimator<N>::calcProbs_pred(
  const boost::array<bool, N>& cpg_states,
  const boost::array<double, N>& cpg_subphases)
{
  Eigen::Matrix<double, N, 1> res;
  for (size_t l = 0; l < N; ++l)
  {
    if (cpg_states[l])
    {
      const auto erf1 = erf((cpg_subphases[l] - 0) / erfden_pred_);
      const auto erf2 = erf((1 - cpg_subphases[l]) / erfden_pred_);
      res(l) = 0.5 * (erf1 + erf2);
    }
    else
    {
      const auto erf1 = erf((0 - cpg_subphases[l]) / erfden_pred_);
      const auto erf2 = erf((cpg_subphases[l] - 1) / erfden_pred_);
      res(l) = 0.5 * (2 + erf1 + erf2);
    }
  }
  return res;
}
}  // namespace tobas_legged_tools
