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
  VectorXd x_scale = VectorXd::Ones(2);

  sqp.initialize(x0, x_scale, f, g, h, dfdx, dgdx, dhdx, dFdx, dGdx, dHdx);

  if (sqp.solve() < 0)
  {
    cerr << sqp.errorMessage() << endl;
    return EXIT_FAILURE;
  }

  cout << "Optimal solution: " << sqp.optimal().transpose() << endl;
  cout << "Number of iterations: " << sqp.iterations() << endl;

  return EXIT_SUCCESS;
}
