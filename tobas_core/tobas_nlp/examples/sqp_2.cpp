/**
 * min  x0 * x1 * (x0 + x1 + x2) + x2
 * s.t. x0^2 + x1^2 + x2^2 + x3^2 = 04
 *      x0 * x1 * x2 * x3 >= 25
 *      1 <= x0, x1, x2, x3 <= 5
 */

#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_nlp/sqp.hpp>

using namespace std;
using namespace Eigen;

double f(const VectorXd& x)
{
  return x(0) * x(3) * (x(0) + x(1) + x(2)) + x(2);
}

VectorXd g(const VectorXd& x)
{
  VectorXd res(9);
  res(0) = 25 - x(0) * x(1) * x(2) * x(3);
  res(1) = -x(0) - 1;
  res(2) = -x(1) - 1;
  res(3) = -x(2) - 1;
  res(4) = -x(3) - 1;
  res(5) = x(0) - 5;
  res(6) = x(1) - 5;
  res(7) = x(2) - 5;
  res(8) = x(3) - 5;
  return res;
}

VectorXd h(const VectorXd& x)
{
  VectorXd res(1);
  res(0) = math::sqr(x(0)) + math::sqr(x(1)) + math::sqr(x(2)) + math::sqr(x(3)) - 40;
  return res;
}

RowVectorXd dfdx(const VectorXd& x)
{
  RowVectorXd res(4);
  res(0) = x(3) * (2 * x(0) + x(1) + x(2));
  res(1) = x(0) * x(3);
  res(2) = x(0) * x(3) + 1;
  res(3) = x(0) * (x(0) + x(1) + x(2));
  return res;
}

MatrixXd dgdx(const VectorXd& x)
{
  MatrixXd res(9, 4);

  res(0, 0) = -x(1) * x(2) * x(3);
  res(0, 1) = -x(0) * x(2) * x(3);
  res(0, 2) = -x(0) * x(1) * x(3);
  res(0, 3) = -x(0) * x(1) * x(2);

  res.block(1, 0, 4, 4).setZero();
  res.block(1, 0, 4, 4).diagonal().fill(-1);
  res.block(5, 0, 4, 4).setZero();
  res.block(5, 0, 4, 4).diagonal().fill(1);

  return res;
}

MatrixXd dhdx(const VectorXd& x)
{
  MatrixXd res(1, 4);
  res(0, 0) = 2 * x(0);
  res(0, 1) = 2 * x(1);
  res(0, 2) = 2 * x(2);
  res(0, 3) = 2 * x(3);
  return res;
}

MatrixXd dFdx(const VectorXd& x)
{
  MatrixXd res(4, 4);
  res(0, 0) = 2 * x(3);
  res(0, 1) = x(3);
  res(0, 2) = x(3);
  res(0, 3) = 2 * x(0) + x(1) + x(2);
  res(1, 0) = x(3);
  res(1, 1) = 0;
  res(1, 2) = 0;
  res(1, 3) = x(0);
  res(2, 0) = x(3);
  res(2, 1) = 0;
  res(2, 2) = 0;
  res(2, 3) = x(0);
  res(3, 0) = 2 * x(0) + x(1) + x(2);
  res(3, 1) = x(0);
  res(3, 2) = x(0);
  res(3, 3) = 0;
  return res;
}

Tensor3Xd dGdx(const VectorXd& x)
{
  Tensor3Xd res(9, 4, 4);
  res.setZero();

  res(0, 0, 0) = 0;
  res(0, 0, 1) = -x(2) * x(3);
  res(0, 0, 2) = -x(1) * x(3);
  res(0, 0, 3) = -x(1) * x(2);

  res(0, 1, 0) = -x(2) * x(3);
  res(0, 1, 1) = 0;
  res(0, 1, 2) = -x(0) * x(3);
  res(0, 1, 3) = -x(0) * x(2);

  res(0, 2, 0) = -x(1) * x(3);
  res(0, 2, 1) = -x(0) * x(3);
  res(0, 2, 2) = 0;
  res(0, 2, 3) = -x(0) * x(1);

  res(0, 3, 0) = -x(1) * x(2);
  res(0, 3, 1) = -x(0) * x(2);
  res(0, 3, 2) = -x(0) * x(1);
  res(0, 3, 3) = 0;

  return res;
}

Tensor3Xd dHdx(const VectorXd&)
{
  Tensor3Xd res(1, 4, 4);
  res.setZero();

  res(0, 0, 0) = 2;
  res(0, 1, 1) = 2;
  res(0, 2, 2) = 2;
  res(0, 3, 3) = 2;

  return res;
}

int main()
{
  nlp::SQP sqp;

  VectorXd x0(4);
  x0 << 1, 5, 5, 1;

  sqp.initialize(x0, f, g, h, dfdx, dgdx, dhdx, dFdx, dGdx, dHdx);

  if (sqp.solve() < 0) {
    cerr << sqp.errorMessage() << endl;
    return EXIT_FAILURE;
  }

  const auto& x_opt = sqp.optimal();

  cout << "Optimal solution: " << x_opt.transpose() << endl;
  cout << "Number of iterations: " << sqp.iterations() << endl;  // scipy.optimize.minimizeだと5
  cout << "----------" << endl;
  cout << "f(x*) = " << f(x_opt) << endl;  // scipy.optimize.minimizeだと17.014017247073575
  cout << "g(x*) = " << g(x_opt).transpose() << endl;
  cout << "h(x*) = " << h(x_opt).transpose() << endl;

  return EXIT_SUCCESS;
}
