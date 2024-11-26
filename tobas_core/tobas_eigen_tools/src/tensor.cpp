#include "../include/tobas_eigen_tools/tensor.hpp"

namespace Eigen
{
MatrixXd operator*(const RowVectorXd& lhs, const Tensor3Xd& rhs)
{
  assert(lhs.size() == rhs.dimension(0));
  const array<IndexPair<int>, 1> dims = { IndexPair<int>(0, 0) };
  const Tensor2Xd prod = TensorMap<const Tensor1Xd>(lhs.data(), lhs.size()).contract(rhs, dims);
  return Map<const MatrixXd>(prod.data(), prod.dimension(0), prod.dimension(1));
}
}  // namespace Eigen
