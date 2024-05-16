#pragma once

#include <Eigen/Core>

#include <tobas_eigen_tools/typedef.hpp>

#include "./chainiksolver.hpp"
#include "./chain.hpp"

namespace tobas_kdl
{
/**
 * \brief Solver for the inverse position kinematics that uses Levenberg-Marquardt.
 *
 * The robustness and speed of this solver is improved in several ways:
 *   - by using a Levenberg-Marquardt method that automatically adapts the damping when
 *     computing the inverse damped least squares inverse velocity kinematics.
 *   - by using an internal implementation of forward position kinematics and the
 *     Jacobian kinematics.  This implementation is more numerically robust,
 *     is able to cache previous computations, and implements an \f$ \mathcal{O}(N) \f$
 *     algorithm for the computation of the Jacobian (with \f$N\f$, the number of joints, and for
 *     a fixed size task space).
 *   - by providing a way to specify the weights in task space, you can weigh rotations wrt
 translations.
 *     This is important e.g. to specify that rotations do not matter for the problem at hand, or to
 *     specify how important you judge rotations w.r.t. translations, typically in S.I.-units,
 ([m],[rad]),
 *     the rotations are over-specified, this can be avoided using the weight matrix. <B>Weights
 also
 *     make the solver more robust </B>.
 *   - only the constructors call <B>memory allocation</B>.
 *
 * De general principles behind the optimisation is inspired on:
 *   Jorge Nocedal, Stephen J. Wright, Numerical Optimization,Springer-Verlag New York, 1999.
 */
class ChainIkSolverPos_LM : public ChainIkSolverPos
{
  using super = ChainIkSolverPos;

public:
  static constexpr int E_GRADIENT_JOINTS_TOO_SMALL = -100;
  static constexpr int E_INCREMENT_JOINTS_TOO_SMALL = -101;

  static constexpr size_t kDefaultMaxIter = 500;
  static constexpr double kDefaultEpsilonCart = 1e-5;
  static constexpr double kDefaultEpsilonJnt = 1e-15;
  static constexpr double kDefaultWeightPos = 1.;
  static constexpr double kDefaultWeightRot = 0.01;

  /**
   * \brief constructs an ChainIkSolverPos_LMA solver.
   *
   * The default parameters are chosen to be applicable to industrial-size robots
   * (e.g. 0.5 to 3 meters range in task space), with an accuracy that is more then
   * sufficient for typical industrial applications.
   *
   * Weights are applied in task space, i.e. the kinematic solver minimizes:
   * \f$ E = \Delta \mathbf{x}^T \mathbf{L} \mathbf{L}^T \Delta \mathbf{x} \f$, with
   * \f$\mathbf{L}\f$ a diagonal matrix.
   *
   * \param chain specifies the kinematic chain.
   * \param L specifies the "square root" of the weight (diagonal) matrix in task space. This
   * diagonal matrix is specified as a vector. \param eps_cart specifies the desired accuracy in
   * task space; <B>after</B> weighing with the weight matrix, it is applied on \f$E\f$. \param
   * max_iter specifies the maximum number of iterations. \param eps_jnt specifies that the
   * algorithm has to stop when the computed joint angle increments are smaller then _eps_joints.
   * This is to avoid unnecessary computations up to _maxiter when the joint angle increments are so
   * small that they effectively (in floating point) do not change the joint angles any more.  The
   * default is a few digits above numerical accuracy.
   */
  explicit ChainIkSolverPos_LM(const Chain& chain);

  virtual void updateInternalDataStructures() override;

  /**
   * \brief computes the inverse position kinematics.
   *
   * \param q_init initial joint position.
   * \param T_base_goal goal position expressed with respect to the robot base.
   * \param q_out  joint position that achieves the specified goal position (if successful).
   * \return E_NOERROR if successful,
   *         E_GRADIENT_JOINTS_TOO_SMALL the gradient of \f$ E \f$ towards the joints is to small,
   *         E_INCREMENT_JOINTS_TOO_SMALL if joint position increments are to small,
   *         E_MAX_ITER_EXCEEDED if number of iterations is exceeded.
   */
  virtual int CartToJnt(const JntArray& q_init, const Frame& T_base_goal) override;

  bool setMaxIter(const size_t& max_iter);
  bool setEpsilonCart(const double& eps_cart);
  bool setEpsilonJnt(const double& eps_jnt);
  bool setWeight(const Eigen::Vector6d& L);

  /**
   * \brief for internal use only.
   * Only exposed for test and diagnostic purposes.
   */
  void displayJacobian(const JntArray& jval);

private:
  // additional specification of the inverse position kinematics problem:
  size_t max_iter_ = kDefaultMaxIter;
  double eps_cart_ = kDefaultEpsilonCart;
  double eps_jnt_ = kDefaultEpsilonJnt;
  Eigen::Vector6d L_;

  // state of compute_fwdpos and compute_jacobian.
  // need 2 vectors because of the somewhat strange definition of segment.hpp
  // you could also recompute jointtip out of jointroot,
  // but then you'll need more expensive cos/sin functions.
  std::vector<Frame> T_base_jointroot_;
  std::vector<Frame> T_base_jointtip_;

  Eigen::JacobiSVD<Eigen::Matrix6Xd> svd_;
  Eigen::Matrix6Xd jac_;  // the last value for the Jacobian after an execution of compute_jacobian.
  Eigen::VectorXd grad_;  // the gradient of the error criterion after an execution of CartToJnt.
  // the last value for the position of the tip of the robot (head) with respect to the base,
  // after an execution of compute_jacobian.
  Frame T_base_head_;

  /**
   * \brief for internal use only.
   *
   * Only exposed for test and diagnostic purposes.
   */
  void computeFwdPos(const Eigen::VectorXd& q);

  /**
   * \brief for internal use only.
   * Only exposed for test and diagnostic purposes.
   * compute_fwdpos(q) should always have been called before.
   */
  void computeJacobian(const Eigen::VectorXd& q);

  void enforceJointLimits(Eigen::VectorXd& q);
};
}  // namespace tobas_kdl
