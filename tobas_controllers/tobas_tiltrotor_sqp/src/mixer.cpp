#include <ranges>

#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_tiltrotor_sqp/mixer.hpp"

#define E3 Matrix3d::Identity()

using namespace std;
using namespace Eigen;
namespace et = eigen_tools;

namespace tobas
{
TiltRotorMixer::TiltRotorMixer(const Drone& drone, const kdl::Tree& tree)
  : drone_(drone), tree_(tree), fk_solver_(tree), inertia_solver_(tree), np_mixer_(drone, tree)
{
  if (drone_.numRotors() > 0 && tree_.getNrOfJoints() > 0)
    updateInternalDataStructures();
}

void TiltRotorMixer::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  np_mixer_.updateInternalDataStructures();

  resetTensors();
  initializeSQP();
  updateWeight();
}

bool TiltRotorMixer::solve(
  const double& cur_voltage,
  const kdl::JntArray& cur_q,
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_acc_W,
  const kdl::Vector& tar_dgyro_B)
{
  assert(cur_voltage > 0);

  // 順運動学を計算
  if (fk_solver_.JntToCart(cur_q) < 0)
  {
    cerr << "Forward kinematics failed: " << fk_solver_.errorMessage() << endl;
    return false;
  }

  // 質量特性を計算
  if (inertia_solver_.JntToCart(cur_q) < 0)
  {
    cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << endl;
    return false;
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();
  const auto& mass = inertia.getMass();

  for (const auto& [idx, rotor_it] : views::enumerate(drone_.rotors))
  {
    const auto& rotor = rotor_it.second;

    // Update B
    const auto d = rotor.sign();
    const auto& cm = rotor.moment_constant;
    const auto& B_Pos_B2P = fk_solver_.getFrame(rotor.link_name).p;
    const auto r = B_Pos_B2P - B_Pos_B2G;
    B_.block<3, 3>(3, 3 * idx) = et::skew(r.data) - (d * cm) * E3;

    // Update ci0
    const auto nr = drone_.numRotors();
    if (!rotor.tilt_joint_name.empty())
    {
      const auto& tilt_joint = drone_.joints.at(rotor.tilt_joint_name);
      ci0_(idx) = tilt_joint.min_pos;
      ci0_(nr + idx) = -tilt_joint.max_pos;
    }
    ci0_(2 * nr + idx) = rotor.minThrust(cur_voltage);
    ci0_(3 * nr + idx) = -rotor.maxThrust(cur_voltage);
  }

  // Update d
  const kdl::Vector grav_W(0, 0, -tobas_std::kGravity);
  d_.head<3>() = (mass * cur_rot.inverse(tar_acc_W - grav_W)).data;
  d_.tail<3>() = (I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B)).data;

  // SQPを解く
  if (sqp_.solve() < 0)
  {
    cerr << "SQP failed: " << sqp_.errorMessage() << endl;
    return false;
  }

  return true;
}

VectorXd TiltRotorMixer::getThrusts() const
{
  return sqp_.optimal().tail(drone_.numRotors());
}

VectorXd TiltRotorMixer::getTiltAngles() const
{
  return sqp_.optimal().head(drone_.numRotors());
}

bool TiltRotorMixer::setLinearWeight(double p)
{
  if (p <= 0.)
  {
    cerr << "Linear weight must be positive." << endl;
    return false;
  }

  linear_weight_ = p;
  updateWeight();
  return true;
}

bool TiltRotorMixer::setAngularWeight(double p)
{
  if (p <= 0.)
  {
    cerr << "Angular weight must be positive." << endl;
    return false;
  }

  angular_weight_ = p;
  updateWeight();
  return true;
}

bool TiltRotorMixer::setThrustWeight(double p)
{
  if (p <= 0.)
  {
    cerr << "Thrust weight must be positive." << endl;
    return false;
  }

  thrust_weight_ = p;
  updateWeight();
  return true;
}

void TiltRotorMixer::resetTensors()
{
  const auto nr = drone_.numRotors();

  R_.resize(nr);

  B_.conservativeResize(NoChange, 3 * nr);
  for (size_t i = 0; i < nr; ++i)
    B_.block<3, 3>(0, 3 * i).setIdentity();

  N_.conservativeResize(3 * nr, nr);
  N_.setZero();

  dN_dtheta_.resize(3 * nr, nr, nr);
  dN_dtheta_.setZero();

  dN_dtheta_2_.resize(3 * nr, nr, nr, nr);
  dN_dtheta_2_.setZero();

  Ci_.conservativeResize(4 * nr, 2 * nr);
  Ci_.setZero();
  Ci_.block(0, 0, nr, nr).diagonal().setConstant(-1);
  Ci_.block(nr, 0, nr, nr).diagonal().setConstant(1);
  Ci_.block(2 * nr, nr, nr, nr).diagonal().setConstant(-1);
  Ci_.block(3 * nr, nr, nr, nr).diagonal().setConstant(1);

  ci0_.conservativeResize(4 * nr);
  ci0_.setZero();

  dCi_dx_.resize(4 * nr, 2 * nr, 2 * nr);
  dCi_dx_.setZero();

  df_dx_.conservativeResize(2 * nr);

  df_dx_2_.conservativeResize(2 * nr, 2 * nr);
}

void TiltRotorMixer::initializeSQP()
{
  const auto q0 = kdl::JntArray::Zero(tree_.getNrOfJoints());
  const auto R0 = kdl::Rotation::Identity();
  const auto v0 = kdl::Vector::Zero();
  if (!np_mixer_.solve(drone_.battery.nominal_voltage, q0, R0, v0, v0, v0))
    throw runtime_error("Failed to solve Non-planar mixer.");

  const auto nr = drone_.numRotors();
  VectorXd x0(2 * nr);
  x0.head(nr).setZero();
  x0.tail(nr) = np_mixer_.getThrusts();

  const auto _f = bind(&self::f, this, std::placeholders::_1);
  const auto _g = bind(&self::g, this, std::placeholders::_1);
  const auto _h = bind(&self::h, this, std::placeholders::_1);
  const auto _dfdx = bind(&self::dfdx, this, std::placeholders::_1);
  const auto _dgdx = bind(&self::dgdx, this, std::placeholders::_1);
  const auto _dhdx = bind(&self::dhdx, this, std::placeholders::_1);
  const auto _dFdx = bind(&self::dFdx, this, std::placeholders::_1);
  const auto _dGdx = bind(&self::dGdx, this, std::placeholders::_1);
  const auto _dHdx = bind(&self::dHdx, this, std::placeholders::_1);
  sqp_.initialize(x0, _f, _g, _h, _dfdx, _dgdx, _dhdx, _dFdx, _dGdx, _dHdx);
}

void TiltRotorMixer::updateWeight()
{
  if (drone_.numRotors() == 0)
    return;

  if (inertia_solver_.JntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints())) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto& I = inertia.getRotationalInertia();

  const auto linear_scale = mass * kAccelScale;                               // [N]
  const auto angular_scale = (I.trace() / 3) * kDGyroScale;                   // [Nm]
  const auto thrust_scale = mass * tobas_std::kGravity / drone_.numRotors();  // [N]

  Q_.diagonal().head<3>().fill(linear_weight_ / math::sqr(linear_scale));
  Q_.diagonal().tail<3>().fill(angular_weight_ / math::sqr(angular_scale));
  R_.diagonal().fill(thrust_weight_ / math::sqr(thrust_scale));
}

double TiltRotorMixer::f(const VectorXd& x)
{
  const auto [theta, tau] = splitState(x);
  const auto e = calc_e(theta, tau);
  return 0.5 * (e.transpose() * Q_ * e).value() + 0.5 * (tau.transpose() * R_ * tau).value();
}

VectorXd TiltRotorMixer::g(const VectorXd& x)
{
  return Ci_ * x + ci0_;
}

VectorXd TiltRotorMixer::h(const VectorXd&)
{
  return VectorXd(0);
}

RowVectorXd TiltRotorMixer::dfdx(const VectorXd& x)
{
  const auto [theta, tau] = splitState(x);

  const auto C = calc_C(theta);
  const Matrix6Xd QC = Q_ * C;

  const auto nr = drone_.numRotors();
  df_dx_.head(nr) = calc_e(theta, tau).transpose() * Q_ * calc_du_dtheta(theta, tau);
  df_dx_.tail(nr) = tau.transpose() * (C.transpose() * QC + R_.toDenseMatrix()) - d_.transpose() * QC;

  return df_dx_;
}

MatrixXd TiltRotorMixer::dgdx(const VectorXd&)
{
  return Ci_;
}

MatrixXd TiltRotorMixer::dhdx(const VectorXd&)
{
  return MatrixXd(0, stateSize());
}

MatrixXd TiltRotorMixer::dFdx(const VectorXd& x)
{
  const auto [theta, tau] = splitState(x);

  const VectorXd e = calc_e(theta, tau);
  const MatrixXd C = calc_C(theta);
  const VectorXd Qe = Q_ * e;
  const MatrixXd QC = Q_ * C;
  const Matrix6Xd du_dtheta = calc_du_dtheta(theta, tau);
  const Tensor3Xd du_dtheta_2 = calc_du_dtheta_2(theta, tau);
  const Tensor3Xd dC_dtheta = calc_dC_dtheta(theta);

  const auto nr = drone_.numRotors();

  df_dx_2_.topLeftCorner(nr, nr) = Qe.transpose().eval() * du_dtheta_2 + du_dtheta.transpose() * Q_ * du_dtheta;
  df_dx_2_.bottomRightCorner(nr, nr) = C.transpose() * QC + R_.toDenseMatrix();

  const MatrixXd d2f_dtheta_dtau = et::shuffle(dC_dtheta, { 2, 1, 0 }) * Qe + du_dtheta.transpose() * QC;
  df_dx_2_.topRightCorner(nr, nr) = d2f_dtheta_dtau;
  df_dx_2_.bottomLeftCorner(nr, nr) = d2f_dtheta_dtau.transpose();

  return df_dx_2_;
}

Tensor3Xd TiltRotorMixer::dGdx(const VectorXd&)
{
  return dCi_dx_;
}

Tensor3Xd TiltRotorMixer::dHdx(const VectorXd&)
{
  const auto state_size = stateSize();
  return Tensor3Xd(0, state_size, state_size);
}

size_t TiltRotorMixer::stateSize() const
{
  return drone_.numRotors() * 2;
}

pair<VectorXd, VectorXd> TiltRotorMixer::splitState(const VectorXd& x) const
{
  assert(static_cast<size_t>(x.size()) == stateSize());

  const VectorXd angles = x.head(drone_.numRotors());
  const VectorXd thrusts = x.tail(drone_.numRotors());

  return { angles, thrusts };
}

Vector6d TiltRotorMixer::calc_e(const VectorXd& theta, const VectorXd& tau)
{
  return calc_u(theta, tau) - d_;
}

Vector6d TiltRotorMixer::calc_u(const VectorXd& theta, const VectorXd& tau)
{
  return calc_C(theta) * tau;
}

Matrix6Xd TiltRotorMixer::calc_C(const VectorXd& theta)
{
  return B_ * calc_N(theta);
}

const MatrixXd& TiltRotorMixer::calc_N(const VectorXd& theta)
{
  for (const auto& [i, rotor_it] : views::enumerate(drone_.rotors))
  {
    const auto& rotor = rotor_it.second;
    const auto& elem = tree_.getSegment(rotor.link_name)->second;

    // TODO: ティルトジョイントがロータジョイントの直接の親じゃない場合にも対応
    if (!rotor.tilt_joint_name.empty())
    {
      const auto& par_elem = elem.parent->second;
      const auto& gpar_elem = par_elem.parent->second;
      const auto& cur_seg = elem.segment;
      const auto& par_seg = par_elem.segment;
      const auto& gpar_seg = gpar_elem.segment;
      const auto& p = par_seg.joint().axis().data;
      const auto& q = cur_seg.joint().axis().data;
      const auto& R_base2gpar = fk_solver_.getFrame(gpar_seg.name()).M.data;
      const Matrix3d R_gpar2par = E3 + et::skew2(p) * (1 - cos(theta(i))) - et::skew(p) * sin(theta(i));
      const Vector3d axis_B = R_base2gpar * R_gpar2par * q;
      N_.block<3, 1>(3 * i, i) = axis_B;

      if (i == 1)
      {
        cout << "theta: " << theta(i) << endl;
        cout << "p: " << p.transpose() << endl;
        cout << "q: " << q.transpose() << endl;
        cout << "R_base2gpar:\n" << R_base2gpar << endl;
        cout << "R_gpar2par:\n" << R_gpar2par << endl;
        cout << "axis_B: " << axis_B.transpose() << endl;
        cout << "----------" << endl;
      }
    }
    else
    {
      const auto& B_Rot_Par = fk_solver_.getFrame(elem.parent->first).M;
      const auto axis_B = B_Rot_Par * elem.segment.joint().axis();
      N_.block<3, 1>(3 * i, i) = axis_B.data;
    }
  }

  // cout << "theta: " << theta.transpose() << endl;
  // cout << N_ << endl;
  // cout << endl;

  return N_;
}

const Tensor3Xd& TiltRotorMixer::calc_dN_dtheta(const VectorXd& theta)
{
  for (const auto& [i, rotor_it] : views::enumerate(drone_.rotors))
  {
    const auto& rotor = rotor_it.second;
    const auto& elem = tree_.getSegment(rotor.link_name)->second;

    if (!rotor.tilt_joint_name.empty())
    {
      const auto& par_elem = elem.parent->second;
      const auto& gpar_elem = par_elem.parent->second;
      const auto& cur_seg = elem.segment;
      const auto& par_seg = par_elem.segment;
      const auto& gpar_seg = gpar_elem.segment;
      const auto& p = par_seg.joint().axis().data;
      const auto& q = cur_seg.joint().axis().data;
      const auto& R = fk_solver_.getFrame(gpar_seg.name()).M.data;
      const Vector3d dn_dtheta = R * (et::skew2(p) * sin(theta(i)) - et::skew(p) * cos(theta(i))) * q;
      et::setVectorX(dN_dtheta_, dn_dtheta, { 3 * (int)i, (int)i, (int)i });
    }
  }

  return dN_dtheta_;
}

const Tensor4Xd& TiltRotorMixer::calc_dN_dtheta_2(const VectorXd& theta)
{
  for (const auto& [i, rotor_it] : views::enumerate(drone_.rotors))
  {
    const auto& rotor = rotor_it.second;
    const auto& elem = tree_.getSegment(rotor.link_name)->second;

    if (!rotor.tilt_joint_name.empty())
    {
      const auto& par_elem = elem.parent->second;
      const auto& gpar_elem = par_elem.parent->second;
      const auto& cur_seg = elem.segment;
      const auto& par_seg = par_elem.segment;
      const auto& gpar_seg = gpar_elem.segment;
      const auto& p = par_seg.joint().axis().data;
      const auto& q = cur_seg.joint().axis().data;
      const auto& R = fk_solver_.getFrame(gpar_seg.name()).M.data;
      const Vector3d dn_dtheta_2 = R * (et::skew2(p) * cos(theta(i)) + et::skew(p) * sin(theta(i))) * q;
      et::setVectorX(dN_dtheta_2_, dn_dtheta_2, { 3 * (int)i, (int)i, (int)i, (int)i });
    }
  }

  return dN_dtheta_2_;
}

Matrix6Xd TiltRotorMixer::calc_du_dtheta(const VectorXd& theta, const VectorXd& tau)
{
  return et::shuffle(calc_dC_dtheta(theta), { 0, 2, 1 }) * tau;
}

Tensor3Xd TiltRotorMixer::calc_du_dtheta_2(const VectorXd& theta, const VectorXd& tau)
{
  return et::shuffle(calc_dC_dtheta_2(theta), { 0, 3, 2, 1 }) * tau;
}

Tensor3Xd TiltRotorMixer::calc_dC_dtheta(const VectorXd& theta)
{
  return B_ * calc_dN_dtheta(theta);
}

Tensor4Xd TiltRotorMixer::calc_dC_dtheta_2(const VectorXd& theta)
{
  return B_ * calc_dN_dtheta_2(theta);
}
}  // namespace tobas
