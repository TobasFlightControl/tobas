#pragma once

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_eigen_tools/tensor.hpp>
#include <tobas_nlp/sqp.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_tools/np_mixer_qp.hpp>

namespace tobas
{
/**
 * @brief ティルトロータマルチコプターのミキシングをSQPで解く． (memo: 3-12)
 */
class TiltRotorMixer_SQP
{
  using self = TiltRotorMixer_SQP;

public:
  explicit TiltRotorMixer_SQP(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures();

  bool solve(
    const double& cur_voltage,
    const kdl::JntArray& cur_q,
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_acc_W,
    const kdl::Vector& tar_dgyro_B);

  double getThrust(size_t idx) const;
  double getTiltAngle(size_t idx) const;

  bool setLinearWeight(double p);
  bool setAngularWeight(double p);
  bool setThrustWeight(double p);

private:
  struct Config
  {
    double linear_weight = 1.;
    double angular_weight = 1.;
    double thrust_weight = 1e-6;
  } cfg_;

  const Drone& drone_;
  const kdl::Tree& tree_;

  kdl::TreeJointParser joint_parser_;
  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;
  NonPlanarMixer_QP np_mixer_;

  nlp::SQP sqp_;

  Eigen::Diagonal6d Q_;  // EoMの重み
  Eigen::DiagonalXd R_;  // 推力の重み
  Eigen::Matrix6Xd B_;
  Eigen::Vector6d d_;
  Eigen::MatrixXd N_;
  Eigen::Tensor3Xd dN_dtheta_;
  Eigen::Tensor4Xd dN_dtheta_2_;
  Eigen::MatrixXd Ci_;
  Eigen::VectorXd ci0_;
  Eigen::Tensor3Xd dCi_dx_;
  Eigen::RowVectorXd df_dx_;
  Eigen::MatrixXd df_dx_2_;

  void resetTensors();
  bool initializeSQP();
  bool updateWeight();

  // SQPに渡す関数
  double f(const Eigen::VectorXd& x);
  Eigen::VectorXd g(const Eigen::VectorXd& x);
  Eigen::VectorXd h(const Eigen::VectorXd& x);
  Eigen::RowVectorXd dfdx(const Eigen::VectorXd& x);
  Eigen::MatrixXd dgdx(const Eigen::VectorXd& x);
  Eigen::MatrixXd dhdx(const Eigen::VectorXd& x);
  Eigen::MatrixXd dFdx(const Eigen::VectorXd&);
  Eigen::Tensor3Xd dGdx(const Eigen::VectorXd&);
  Eigen::Tensor3Xd dHdx(const Eigen::VectorXd& x);

  size_t stateSize() const;
  std::pair<Eigen::VectorXd, Eigen::VectorXd> splitState(const Eigen::VectorXd& x) const;

  Eigen::Vector6d calc_e(const Eigen::VectorXd& theta, const Eigen::VectorXd& tau);
  Eigen::Vector6d calc_u(const Eigen::VectorXd& theta, const Eigen::VectorXd& tau);
  Eigen::Matrix6Xd calc_C(const Eigen::VectorXd& theta);
  const Eigen::MatrixXd& calc_N(const Eigen::VectorXd& theta);
  const Eigen::Tensor3Xd& calc_dN_dtheta(const Eigen::VectorXd& theta);
  const Eigen::Tensor4Xd& calc_dN_dtheta_2(const Eigen::VectorXd& theta);
  Eigen::Matrix6Xd calc_du_dtheta(const Eigen::VectorXd& theta, const Eigen::VectorXd& tau);
  Eigen::Tensor3Xd calc_du_dtheta_2(const Eigen::VectorXd& theta, const Eigen::VectorXd& tau);
  Eigen::Tensor3Xd calc_dC_dtheta(const Eigen::VectorXd& theta);
  Eigen::Tensor4Xd calc_dC_dtheta_2(const Eigen::VectorXd& theta);
};
}  // namespace tobas
