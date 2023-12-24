#include <tobas_quadprog/quadprogpp.hpp>
#include <tobas_quadprog/qpoases.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_quadprog/primal_dual_interior_point.hpp>

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
  const auto quadprog_sol = quadprog.solve();
  cout << "QuadProg++ solution: " << quadprog_sol.transpose() << endl;

  quadprog::QpOasesSolver qpoases;
  qpoases.problem = problem;
  qpoases.x_scale = x_scale;
  const auto qpoases_sol = qpoases.solve();
  cout << "qpOASES solution: " << qpoases_sol.transpose() << endl;

  quadprog::DualActiveSetSolver das;
  das.problem = problem;
  das.x_scale = x_scale;
  const auto das_sol = das.solve();
  cout << "DualActiveSet solution: " << das_sol.transpose() << endl;

  quadprog::PrimalDualInteriorPointSolver ipm;
  ipm.problem = problem;
  ipm.x_scale = x_scale;
  const auto ipm_sol = ipm.solve();
  cout << "PrimalDualInteriorPointSolver solution: " << ipm_sol.transpose() << endl;
}
