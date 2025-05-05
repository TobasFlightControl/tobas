#include "../include/tobas_quadprog/dual_active_set.hpp"

#include <iostream>

#include <eigen3/Eigen/Cholesky>

#include <tobas_math/core.hpp>

#define EPS numeric_limits<double>::epsilon()
#define INF numeric_limits<double>::infinity()
#define TOL_FACTOR 100.

// #define TRACE_SOLVER

using namespace std;
using namespace Eigen;

namespace quadprog
{
DualActiveSetSolver::DualActiveSetSolver() : super()
{
}

bool DualActiveSetSolver::solve()
{
  checkProblemValidity();

  // Scaling
  const auto scaled = scaleProblem();

#ifdef TRACE_SOLVER
  cout << "Start to solve QP" << endl;
  cout << "P:\n" << scaled.P << endl;
  cout << "q:\n" << scaled.q.transpose() << endl;
  cout << "G:\n" << scaled.G << endl;
  cout << "h:\n" << scaled.h.transpose() << endl;
  cout << "A:\n" << scaled.A << endl;
  cout << "b:\n" << scaled.b.transpose() << endl;
#endif

  // Problem size
  n_ = scaled.P.cols();
  p_ = scaled.G.rows();
  m_ = scaled.A.rows();
  const auto num_constraints = p_ + m_;
  R_.conservativeResize(n_, n_);
  J_.conservativeResize(n_, n_);
  s_.conservativeResize(m_);
  z_.conservativeResize(n_);
  r_.conservativeResize(num_constraints);
  d_.conservativeResize(n_);
  np_.conservativeResize(n_);
  x_.conservativeResize(n_);
  u_.conservativeResize(num_constraints);
  A_.conservativeResize(num_constraints);
  x_old_.conservativeResize(n_);
  u_old_.conservativeResize(num_constraints);
  A_old_.conservativeResize(num_constraints);
  iai_.conservativeResize(num_constraints);
  iaexcl_.resize(num_constraints);

  /* Preprocessing phase */

  // Decompose the matrix P in the form L L^T
  const LLT<MatrixXd> llt(scaled.P);
  if (llt.info() == NumericalIssue) {
    error_msg_ = "Cholesky decomposition failed.";
    return false;
  }

#ifdef TRACE_SOLVER
  cout << "LL^T:\n" << llt.matrixLLT() << endl;
#endif

  // Compute the inverse of the factorized matrix U^-1, this is the initial value for H
  J_ = llt.matrixU().solve(MatrixXd::Identity(n_, n_));

#ifdef TRACE_SOLVER
  cout << "J:\n" << J_ << endl;
#endif

  iq_ = 0;
  c_ = scaled.P.trace() * J_.trace();  // An estimate for cond(P)
  R_norm_ = 1.;                        // This variable will hold the norm of the matrix R
  R_.setZero();
  d_.setZero();

  // Find the unconstrained minimizer of the quadratic form 0.5 * x^T P x + q^T x
  // This is a feasible point in the dual space: x = -P^-1 * q
  x_ = -llt.solve(scaled.q);

#ifdef TRACE_SOLVER
  cout << "Unconstrained solution:\n" << x_.transpose() << endl;
#endif

  // Add equality constraints to the active set A
  for (Index i = 0; i < p_; ++i) {
    np_ = -scaled.G.row(i);
    d_ = J_.transpose() * np_;
    z_ = J_.rightCols(n_ - iq_) * d_.tail(n_ - iq_);
    update_r();

#ifdef TRACE_SOLVER
    cout << "R:\n" << R_.topLeftCorner(n_, iq_) << endl;
    cout << "z:\n" << z_.transpose() << endl;
    cout << "r:\n" << r_.head(iq_).transpose() << endl;
    cout << "d:\n" << d_.transpose() << endl;
#endif

    // Compute full step length t2: i.e., the minimum step in primal space
    // s.t. the constraint becomes feasible
    const auto t2 = z_.dot(z_) > EPS ? (-np_.dot(x_) - scaled.h(i)) / z_.dot(np_) : 0.;

    // Set x = x + t2 * z
    x_ += t2 * z_;

    // Set u = u - t2 * r
    u_(iq_) = t2;
    u_.head(iq_) -= t2 * r_.head(iq_);

    // To distinguish between equality constraints and inequality constraints,
    // the indexes of equality constraints are stored as negative values.
    A_(i) = -i - 1;

    if (!addConstraint()) {
      error_msg_ = "Constraints are linearly dependent.";
      return false;
    }
  }

  // Set iai = K \ A
  for (Index i = 0; i < m_; ++i) {
    iai_(i) = i;
  }

  state_t state = CHOOSE_VIOLATED_CONSTRAINT;
  while (true) {
    switch (state) {
      case CHOOSE_VIOLATED_CONSTRAINT: {
#ifdef TRACE_SOLVER
        cout << "x:\n" << x_.transpose() << endl;
#endif

        for (Index i = p_; i < iq_; ++i) {
          iai_(A_(i)) = -1;
        }

        ss_ = 0.;
        ip_ = 0;  // ip will be the index of the chosen violated constraint
        for (Index i = 0; i < m_; ++i) {
          iaexcl_[i] = true;
        }

        // Compute s[x] = b - A * x for all elements of K \ A
        s_.head(m_) = scaled.b - scaled.A * x_;

#ifdef TRACE_SOLVER
        cout << "s:\n" << s_.head(m_).transpose() << endl;
#endif

        const auto psi = s_.head(m_).cwiseMin(0.).sum();  // Sum of all infeasibilities
        if (fabs(psi) <= m_ * EPS * c_ * TOL_FACTOR) {
          // Numerically there are no infeasibilities anymore
          x_opt_ = x_.cwiseProduct(x_scale);
          return true;
        }

        // Save old values for u, A, and x
        u_old_.head(iq_) = u_.head(iq_);
        A_old_.head(iq_) = A_.head(iq_);
        x_old_ = x_;

        state = CHECK_FEASIBILITY;
        break;
      }
      case CHECK_FEASIBILITY: {
        for (Index i = 0; i < m_; ++i) {
          if (s_(i) < ss_ && iai_(i) != -1 && iaexcl_[i]) {
            ss_ = s_(i);
            ip_ = i;
          }
        }
        if (ss_ >= 0.) {
          x_opt_ = x_.cwiseProduct(x_scale);
          return true;
        }

        // Set np = n(ip)
        np_ = -scaled.A.row(ip_);
        // Set u = [u 0]^T
        u_(iq_) = 0.;
        // Add ip to the active set A
        A_(iq_) = ip_;

#ifdef TRACE_SOLVER
        cout << "Trying with constraint " << ip_ << endl;
        cout << "np:\n" << np_.transpose() << endl;
#endif

        state = DETERMINE_STEP_DIRECTION;
        break;
      }
      case DETERMINE_STEP_DIRECTION: {
        // Compute z = H np: the step direction in the primal space (through J, see the paper)
        d_ = J_.transpose() * np_;
        z_ = J_.rightCols(n_ - iq_) * d_.tail(n_ - iq_);
        // Compute N * np (if q > 0): the negative of the step direction in the dual space
        update_r();

#ifdef TRACE_SOLVER
        cout << "Step direction z" << endl;
        cout << "z:\n" << z_.transpose() << endl;
        cout << "r:\n" << r_.head(iq_ + 1).transpose() << endl;
        cout << "u:\n" << u_.head(iq_ + 1).transpose() << endl;
        cout << "d:\n" << d_.transpose() << endl;
        cout << "A:\n" << A_.head(iq_ + 1).transpose() << endl;
#endif

        // Step 2b: Compute step length
        Index l = 0;

        // Compute t1: partial step length
        // = maximum step in dual space without violating dual feasibility
        auto t1 = INF;
        // Find the index l s.t. it reaches the minimum of u+[x] / r
        for (Index k = p_; k < iq_; ++k) {
          if (r_(k) > 0.) {
            if (u_(k) / r_(k) < t1) {
              t1 = u_(k) / r_(k);
              l = A_(k);
            }
          }
        }

        // Compute t2: full step length
        // = minimum step in primal space such that the constraint ip becomes feasible
        auto t2 = INF;
        if (z_.dot(z_) > EPS)  // i.e. z != 0
        {
          t2 = -s_(ip_) / z_.dot(np_);
          if (t2 < 0)  // Patch suggested by Takano Akio for handling numerical inconsistencies
          {
            t2 = INF;
          }
        }

        // The step is chosen as the minimum of t1 and t2
        const auto t = min(t1, t2);

#ifdef TRACE_SOLVER
        cout << "Step sizes: " << t << " (t1 = " << t1 << ", t2 = " << t2 << ") ";
#endif

        // Step 2c: determine new S-pair and take step:

        // case (i): no step in primal or dual space
        if (t >= INF) {
          error_msg_ = "QPP is infeasible.";
          return false;
        }

        // case (ii): step in dual space
        if (t2 >= INF) {
          // Set u = u + t * [-r 1] and drop constraint l from the active set A
          u_.head(iq_) -= t * r_.head(iq_);
          u_(iq_) += t;
          iai_(l) = l;
          deleteConstraint(l);

#ifdef TRACE_SOLVER
          cout << " in dual space:" << endl;
          cout << "x:\n" << x_.transpose() << endl;
          cout << "z:\n" << z_.transpose() << endl;
          cout << "A:\n" << A_.head(iq_ + 1).transpose() << endl;
#endif

          state = DETERMINE_STEP_DIRECTION;
          break;
        }

        // case (iii): step in primal and dual space

        // Set x = x + t * z
        x_ += t * z_;
        // Set u = u + t * [-r 1]
        u_.head(iq_) -= t * r_.head(iq_);
        u_(iq_) += t;

#ifdef TRACE_SOLVER
        cout << " in both spaces:" << endl;
        cout << "x:\n" << x_.transpose() << endl;
        cout << "u:\n" << u_.head(iq_ + 1).transpose() << endl;
        cout << "r:\n" << r_.head(iq_ + 1).transpose() << endl;
        cout << "A:\n" << A_.head(iq_ + 1).transpose() << endl;
#endif

        if (fabs(t - t2) < EPS) {
#ifdef TRACE_SOLVER
          cout << "Full step has taken " << t << endl;
          cout << "x:\n" << x_.transpose() << endl;
#endif

          // Full step has taken
          // Add constraint ip to the active set
          if (!addConstraint()) {
            iaexcl_[ip_] = false;
            deleteConstraint(ip_);

#ifdef TRACE_SOLVER
            cout << "R:\n" << R_ << endl;
            cout << "A:\n" << A_.head(iq_).transpose() << endl;
            cout << "iai:\n" << iai_.transpose() << endl;
#endif

            for (Index i = 0; i < m_; ++i) {
              iai_(i) = i;
            }
            for (Index i = p_; i < iq_; ++i) {
              A_(i) = A_old_(i);
              u_(i) = u_old_(i);
              iai_(A_(i)) = -1;
            }
            x_ = x_old_;

            state = CHECK_FEASIBILITY;
            break;
          }
          else {
            iai_(ip_) = -1;
          }

#ifdef TRACE_SOLVER
          cout << "R:\n" << R_ << endl;
          cout << "A:\n" << A_.head(iq_).transpose() << endl;
          cout << "iai:\n" << iai_.transpose() << endl;
#endif

          state = CHOOSE_VIOLATED_CONSTRAINT;
          break;
        }

        // A partial step has taken
#ifdef TRACE_SOLVER
        cout << "Partial step has taken " << t << endl;
        cout << "x:\n" << x_.transpose() << endl;
#endif

        // Drop constraint l
        iai_(l) = l;
        deleteConstraint(l);

#ifdef TRACE_SOLVER
        cout << "R:\n" << R_ << endl;
        cout << "A:\n" << A_.head(iq_).transpose() << endl;
#endif

        // update s(ip) = b - A * x
        s_(ip_) = scaled.b(ip_) - scaled.A.row(ip_).dot(x_);

#ifdef TRACE_SOLVER
        cout << "s:\n" << s_.transpose() << endl;
#endif

        state = DETERMINE_STEP_DIRECTION;
        break;
      }
    }
  }
}

VectorXd DualActiveSetSolver::getLagrangeMultipliersEq() const
{
  return u_.head(p_);
}

VectorXd DualActiveSetSolver::getLagrangeMultipliersIneq() const
{
  VectorXd lambda = VectorXd::Zero(m_);
  for (Index i = p_; i < iq_; ++i) {
    lambda(A_(i)) = u_(i);
  }
  return lambda;
}

void DualActiveSetSolver::update_r()
{
  // Set r = R^-1 d
  for (Index i = iq_ - 1; i >= 0; --i) {
    const auto sum = (R_.block(i, i + 1, 1, iq_ - i - 1) * r_.block(i + 1, 0, iq_ - i - 1, 1)).value();
    r_(i) = (d_(i) - sum) / R_(i, i);
  }
}

bool DualActiveSetSolver::addConstraint()
{
#ifdef TRACE_SOLVER
  cout << "Add constraint " << iq_ << "/";
#endif

  // We have to find the Givens rotation which will reduce the element d(j) to zero.
  // If it is already zero, we don't have to do anything, except of decreasing j.
  for (Index j = n_ - 1; j >= iq_ + 1; --j) {
    // The Givens rotation is done with the matrix (cc cs, cs -cc).
    // If cc is one, then element (j) of d is zero compared with element (j - 1).
    // Hence we don't have to do anything.
    // If cc is zero, then we just have to switch column (j) and column (j - 1) of J.
    // Since we only switch columns in J, we have to be careful how we update d
    // depending on the sign of gs.
    // Otherwise we have to apply the Givens rotation to these columns.
    // The (i - 1) element of d has to be updated to h.
    auto cc = d_(j - 1);
    auto ss = d_(j);
    const auto h = distance(cc, ss);
    if (fabs(h) < EPS)  // h == 0
    {
      continue;
    }

    d_(j) = 0.;
    ss = ss / h;
    cc = cc / h;
    if (cc < 0.) {
      cc = -cc;
      ss = -ss;
      d_(j - 1) = -h;
    }
    else {
      d_(j - 1) = h;
    }

    const auto xny = ss / (1. + cc);
    const VectorXd t1 = J_.col(j - 1);
    const VectorXd t2 = J_.col(j);
    J_.col(j - 1) = t1 * cc + t2 * ss;
    J_.col(j) = xny * (t1 + J_.col(j - 1)) - t2;
  }
  // Update the number of constraints added
  ++iq_;

  // To update R we have to put the iq components of the d vector into column iq - 1 of R
  R_.block(0, iq_ - 1, iq_, 1) = d_.head(iq_);

#ifdef TRACE_SOLVER
  cout << iq_ << endl;
  cout << "R:\n" << R_.topLeftCorner(iq_, iq_) << endl;
  cout << "J:\n" << J_ << endl;
  cout << "d:\n" << d_.head(iq_).transpose() << endl;
#endif

  if (fabs(d_(iq_ - 1)) <= EPS * R_norm_) {
    error_msg_ = "Problem degenerate.";
    return false;
  }

  R_norm_ = max(R_norm_, fabs(d_(iq_ - 1)));
  return true;
}

void DualActiveSetSolver::deleteConstraint(const Index& l)
{
#ifdef TRACE_SOLVER
  cout << "Delete constraint " << l << " " << iq_;
#endif

  Index qq = 0;  // Initialize qq just to prevent warnings from smart compilers
  [[maybe_unused]] bool found = false;

  // Find the index qq for active constraint l to be removed
  for (Index i = p_; i < iq_; ++i) {
    if (A_(i) == l) {
      qq = i;
      found = true;
      break;
    }
  }
  assert(found);

  // Remove the constraint from the active set and the duals
  A_.block(qq, 0, iq_ - 1 - qq, 1) = A_.block(qq + 1, 0, iq_ - 1 - qq, 1);
  u_.block(qq, 0, iq_ - 1 - qq, 1) = u_.block(qq + 1, 0, iq_ - 1 - qq, 1);
  R_.block(0, qq, n_, iq_ - 1 - qq) = R_.block(0, qq + 1, n_, iq_ - 1 - qq);
  A_(iq_ - 1) = A_(iq_);
  u_(iq_ - 1) = u_(iq_);
  A_(iq_) = 0;
  u_(iq_) = 0.;
  R_.block(0, iq_ - 1, iq_, 1).setZero();

  // Constraint has been fully removed
  --iq_;

#ifdef TRACE_SOLVER
  cout << "/" << iq_ << endl;
#endif

  if (iq_ == 0) {
    return;
  }

  for (Index j = qq; j < iq_; ++j) {
    auto cc = R_(j, j);
    auto ss = R_(j + 1, j);
    const auto h = distance(cc, ss);
    if (fabs(h) < EPS)  // h == 0
    {
      continue;
    }
    cc = cc / h;
    ss = ss / h;
    R_(j + 1, j) = 0.;
    if (cc < 0.) {
      R_(j, j) = -h;
      cc = -cc;
      ss = -ss;
    }
    else {
      R_(j, j) = h;
    }

    const auto xny = ss / (1. + cc);

    const RowVectorXd r1 = R_.block(j, j + 1, 1, iq_ - j - 1);
    const RowVectorXd r2 = R_.block(j + 1, j + 1, 1, iq_ - j - 1);
    R_.block(j, j + 1, 1, iq_ - j - 1) = r1 * cc + r2 * ss;
    R_.block(j + 1, j + 1, 1, iq_ - j - 1) = xny * (r1 + R_.block(j, j + 1, 1, iq_ - j - 1)) - r2;

    const VectorXd j1 = J_.col(j);
    const VectorXd j2 = J_.col(j + 1);
    J_.col(j) = j1 * cc + j2 * ss;
    J_.col(j + 1) = xny * (J_.col(j) + j1) - j2;
  }
}

double DualActiveSetSolver::distance(const double& a, const double& b)
{
  const auto a1 = fabs(a);
  const auto b1 = fabs(b);
  if (a1 > b1) {
    const auto t = b1 / a1;
    return a1 * sqrt(1. + math::sqr(t));
  }
  else if (b1 > a1) {
    const auto t = a1 / b1;
    return b1 * sqrt(1. + math::sqr(t));
  }
  else {
    return a1 * M_SQRT2;
  }
}
}  // namespace quadprog
