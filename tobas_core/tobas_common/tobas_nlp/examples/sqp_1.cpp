/**
 * min  x0^2 + x0 * x1
 * s.t. x0^3 + x0 * x1 = 100
 *      x0^2 + 4 * x1 >= 50
 *      -100 <= x0, x1 <= 100
 */

#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_nlp/sqp.hpp>

using namespace std;
using namespace Eigen;

double f(const VectorXd& x)
{
  return math::sqr(x(0)) + x(0) * x(1);
}

VectorXd g(const VectorXd& x)
{
  VectorXd res(5);
  res(0) = 50 - math::sqr(x(0)) - 4 * x(1);
  res(1) = -x(0) - 100;
  res(2) = -x(1) - 100;
  res(3) = x(0) - 100;
  res(4) = x(1) - 100;
  return res;
}

VectorXd h(const VectorXd& x)
{
  VectorXd res(1);
  res(0) = math::cube(x(0)) + x(0) * x(1) - 100;
  return res;
}

RowVectorXd dfdx(const VectorXd& x)
{
  RowVectorXd res(2);
  res(0) = 2 * x(0) + x(1);
  res(1) = x(0);
  return res;
}

MatrixXd dgdx(const VectorXd& x)
{
  MatrixXd res(5, 2);
  res(0, 0) = -2 * x(0);
  res(0, 1) = -4;
  res(1, 0) = -1;
  res(1, 1) = 0;
  res(2, 0) = 0;
  res(2, 1) = -1;
  res(3, 0) = 1;
  res(3, 1) = 0;
  res(4, 0) = 0;
  res(4, 1) = 1;
  return res;
}

MatrixXd dhdx(const VectorXd& x)
{
  MatrixXd res(1, 2);
  res(0, 0) = 3 * math::sqr(x(0)) + x(1);
  res(0, 1) = x(0);
  return res;
}

MatrixXd dFdx(const VectorXd&)
{
  MatrixXd res(2, 2);
  res(0, 0) = 2;
  res(0, 1) = 1;
  res(1, 0) = 1;
  res(1, 1) = 0;
  return res;
}

Tensor3Xd dGdx(const VectorXd&)
{
  Tensor3Xd res(5, 2, 2);
  res.setZero();
  res(0, 0, 0) = -2;
  return res;
}

Tensor3Xd dHdx(const VectorXd& x)
{
  Tensor3Xd res(1, 2, 2);
  res(0, 0, 0) = 6 * x(0);
  res(0, 0, 1) = 1;
  res(0, 1, 0) = 1;
  res(0, 1, 1) = 0;
  return res;
}

int main()
{
  nlp::SQP sqp;

  VectorXd x0(2);
  x0 << 5, 5;

  sqp.initialize(x0, f, g, h, dfdx, dgdx, dhdx, dFdx, dGdx, dHdx);

  if (sqp.solve() < 0) {
    cerr << sqp.errorMessage() << endl;
    return EXIT_FAILURE;
  }

  const auto& x_opt = sqp.optimal();

  cout << "Optimal solution: " << x_opt.transpose() << endl;  // 4.04092525, 8.41773078
  cout << "Number of iterations: " << sqp.iterations() << endl;
  cout << "----------" << endl;
  cout << "f(x*) = " << f(x_opt) << endl;
  cout << "g(x*) = " << g(x_opt).transpose() << endl;
  cout << "h(x*) = " << h(x_opt).transpose() << endl;

  return EXIT_SUCCESS;
}
