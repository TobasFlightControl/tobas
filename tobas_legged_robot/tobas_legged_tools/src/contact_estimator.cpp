#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_std_tools/vector.hpp>

#include "../include/tobas_legged_tools/contact_estimator.hpp"

using namespace std;
using namespace Eigen;

namespace lr_tools
{
ContactEstimator::ContactEstimator(const kdl::Tree& tree, const vector<string>& foot_names)
  : tree_(tree), fk_solver_(tree), inertia_solver_(tree), foot_names_(foot_names), nc_(foot_names.size()), states_(nc_)
{
  setPredictionVariance(kDefaultPredictionErfVariance);
  setFootHeightVariance(kDefaultFootHeightErfVariance);
  setContactForceVariance(kDefaultContactForceErfVariance);

  setupKalmanFilter();
  updateInternalDataStructures();
  reset();
}

void ContactEstimator::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  inertia_solver_.JntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints()));
  const auto& mass = inertia_solver_.getInertia().getMass();
  mean_force_ = mass * tobas_std::kGravity / nc_;
}

void ContactEstimator::update(
  const kdl::Frame& T,
  const kdl::JntArray& q,
  const vector<double>& contact_forces,
  const vector<bool>& cpg_states,
  const vector<double>& cpg_subphases)
{
  assert(contact_forces.size() == nc_);
  assert(cpg_states.size() == nc_);
  assert(cpg_subphases.size() == nc_);

  // カルマンフィルタで接触確率を推定
  kf_.y.segment(0, nc_) = calcProbs_height(T, q);
  kf_.y.segment(nc_, nc_) = calcProbs_force(contact_forces);  // TODO: 論文通り接触センサに加えトルクも利用
  kf_.u = calcProbs_pred(cpg_states, cpg_subphases);
  kf_.update();

  // 接触状態を更新 (Fig.10)
  for (size_t l = 0; l < nc_; ++l)
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
        throw runtime_error("Invalid contact state");
      }
    }
  }
}

void ContactEstimator::reset()
{
  const auto ones = VectorXd::Ones(nc_);
  kf_.initialize(ones, ones.asDiagonal());

  tobas_std::fill(states_, Contact);
}

void ContactEstimator::setPredictionVariance(const double& var)
{
  assert(var > 0);
  erfden_pred_ = sqrt(2 * var);
}

void ContactEstimator::setFootHeightVariance(const double& var)
{
  assert(var > 0);
  erfden_height_ = sqrt(2 * var);
}

void ContactEstimator::setContactForceVariance(const double& var)
{
  assert(var > 0);
  erfden_force_ = sqrt(2 * var);
}

bool ContactEstimator::isContact(const size_t& idx) const
{
  return states_[idx] == Contact || states_[idx] == Early;
}

double ContactEstimator::contactProbability(const size_t& idx) const
{
  return kf_.state()(idx);
}

double ContactEstimator::contactProbabilityFromCPG(const size_t& idx) const
{
  return kf_.u(idx);
}

void ContactEstimator::setupKalmanFilter()
{
  kf_.resize(nc_, nc_, nc_ * kNumMeasurements, nc_);

  kf_.ss.A.setZero();
  kf_.ss.B.setIdentity();
  kf_.ss.C.block(0, 0, nc_, nc_).setIdentity();
  kf_.ss.C.block(nc_, 0, nc_, nc_).setIdentity();
  kf_.Bv.setIdentity();

  kf_.Q.setZero();
  kf_.Q.diagonal().fill(kPredictionNoiseVariance);

  kf_.R.setZero();
  kf_.R.diagonal().segment(0, nc_).fill(kFootHeightNoiseVariance);
  kf_.R.diagonal().segment(nc_, nc_).fill(kContactForceNoiseVariance);
}

VectorXd ContactEstimator::calcProbs_height(const kdl::Frame& T, const kdl::JntArray& q)
{
  VectorXd res(nc_);
  for (size_t l = 0; l < nc_; ++l)
  {
    fk_solver_.JntToCart(q, foot_names_[l]);
    const auto F_Pos_FC = T.M * fk_solver_.getFrame().p;
    const auto height = T.p.z() + F_Pos_FC.z();
    res(l) = 0.5 * (1 + erf((kMeanFootHeight - height) / erfden_height_));
  }
  return res;
}

VectorXd ContactEstimator::calcProbs_force(const vector<double>& contact_forces)
{
  VectorXd res(nc_);
  for (size_t l = 0; l < nc_; ++l)
    res(l) = 0.5 * (1 + erf((contact_forces[l] - mean_force_) / erfden_force_));
  return res;
}

VectorXd ContactEstimator::calcProbs_pred(const vector<bool>& cpg_states, const vector<double>& cpg_subphases)
{
  VectorXd res(nc_);
  for (size_t l = 0; l < nc_; ++l)
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
}  // namespace lr_tools
