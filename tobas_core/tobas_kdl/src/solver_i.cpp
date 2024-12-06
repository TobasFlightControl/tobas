#include "../include/tobas_kdl/solver_i.hpp"

using namespace std;

namespace kdl
{
int SolverI::copyError(const SolverI& arg)
{
  error_code_ = arg.errorCode();
  error_msg_ = arg.errorMessage();
  return error_code_;
}

int SolverI::setDefaultError(const int& error_code)
{
  error_code_ = error_code;
  error_msg_ = defaultErrorMessage(error_code);
  return error_code_;
}

string SolverI::defaultErrorMessage(const int& error_code) const
{
  switch (error_code)
  {
    case E_NOERROR:
      return "";  // 正常時はメモリ割り当てを回避しパフォーマンスを向上させるためにエラーメッセージを空文字にしておく．
    case E_NO_CONVERGE:
      return "Failed to converge";
    case E_UNDEFINED:
      return "Undefined value";
    case E_NOT_UP_TO_DATE:
      return "Internal data structures not up to date with Tree or Chain";
    case E_SIZE_MISMATCH:
      return "The size of the input does not match the internal state";
    case E_MAX_ITERATIONS_EXCEEDED:
      return "The maximum number of iterations is exceeded";
    case E_OUT_OF_RANGE:
      return "The requested index is out of range";
    case E_NOT_IMPLEMENTED:
      return "The requested function is not yet implemented";
    case E_SVD_FAILED:
      return "SVD failed";
    case E_NOT_FOUND:
      return "Something is not found";
    case E_NEGATIVE_DELTA_TIME:
      return "Delta time is negative";
    default:
      return "UNKNOWN ERROR";
  }
}
}  // namespace kdl
