#include <iostream>

#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_quadprog/primal_dual_interior_point.hpp>
#include <tobas_quadprog/qpoases.hpp>
#include <tobas_quadprog/quadprogpp.hpp>

using namespace std;
using namespace Eigen;

int main()
{
  quadprog::QuadProgProblem problem(2, 0, 2);
  problem.P << 1., 0., 0., 0.5;
  problem.q << 1.5, 1.;
  problem.A << 1., 1., -1., -1.;
  problem.b << 1., 0.;
  const Vector2d x_scale = Vector2d::Ones();

  quadprog::QuadProgppSolver quadprog;
  quadprog.problem = problem;
  quadprog.x_scale = x_scale;
  if (!quadprog.solve()) {
    cerr << "QuadProgppSolver failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "QuadProg++ solution: " << quadprog.solution().transpose() << endl;

  quadprog::QpOasesSolver qpoases;
  qpoases.problem = problem;
  qpoases.x_scale = x_scale;
  if (!qpoases.solve()) {
    cerr << "QpOasesSolver failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "qpOASES solution: " << qpoases.solution().transpose() << endl;

  quadprog::DualActiveSetSolver das;
  das.problem = problem;
  das.x_scale = x_scale;
  if (!das.solve()) {
    cerr << "DualActiveSetSolver failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "DualActiveSet solution: " << das.solution().transpose() << endl;

  quadprog::PrimalDualInteriorPointSolver ipm;
  ipm.problem = problem;
  ipm.x_scale = x_scale;
  if (!ipm.solve()) {
    cerr << "PrimalDualInteriorPointSolver failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "PrimalDualInteriorPointSolver solution: " << ipm.solution().transpose() << endl;

  return EXIT_SUCCESS;
}
