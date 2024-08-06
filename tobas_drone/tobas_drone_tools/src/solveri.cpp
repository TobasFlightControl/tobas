#include "../include/tobas_drone_tools/solveri.hpp"

namespace tobas
{
int SolverI::updateError(const SolverI& arg)
{
  if (arg.errorCode() < error_code_)
  {
    error_code_ = arg.errorCode();
    error_msg_ = arg.errorMessage();
  }
  return error_code_;
}
}  // namespace tobas
