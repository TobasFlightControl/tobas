#pragma once

#include <cstdint>
#include <functional>

namespace nlp
{
/**
 * @brief 1次元のニュートン法のソルバー．
 *
 * Solve: f(x) = 0
 */
class NewtonSolver1d
{
public:
  enum ErrorCode
  {
    kNoError = 0,
    kMaxIterationExceeded = -1,
    kInfeasible = -2,
  };

  explicit NewtonSolver1d();

  void initialize(std::function<double(double)> f, std::function<double(double)> dfdx);

  ErrorCode solve(double& x);

  ErrorCode errorCode() const;
  const char* errorMessage() const;

  bool setMaximumIterations(size_t max_iter);
  bool setAbsoluteTolerance(double abs_tol);

private:
  ErrorCode error_code_;

  std::function<double(double)> f_;
  std::function<double(double)> dfdx_;

  // Configurations
  size_t max_iter_ = UINT64_MAX;
  double abs_tol_ = 1e-10;
};
}  // namespace nlp
