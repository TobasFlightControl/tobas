#include <tobas_eigen_tools/core.hpp>

#include "../include/tobas_quadprog/utils.hpp"

using namespace std;
using namespace Eigen;

namespace quadprog
{
void matIneqFromRange(const VectorXd& lb, const VectorXd& ub, MatrixXd& A, VectorXd& b, const double inf)
{
  assert(lb.rows() == ub.rows());
  assert(((ub - lb).array() >= 0.).all());

  const auto size = lb.rows();

  const MatrixXd E = MatrixXd::Identity(size, size);
  const auto left = eigen::concat(-E, E, 0);
  const auto right = eigen::concat(-lb, ub, 0);
  const auto is_valid = (right.array().abs() < inf).eval();
  const auto num_valid = is_valid.count();

  A.conservativeResize(num_valid, size);
  b.conservativeResize(num_valid);

  int row = 0;  // 行列不等式の行番号
  for (int i = 0; i < size * 2; ++i)
  {
    if (!is_valid(i))
      continue;
    A.block(row, 0, 1, size) = left.row(i);
    b(row) = right(i);
    ++row;
  }
}
}  // namespace quadprog
