// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/solver_i.hpp"

using namespace std;

namespace tobas
{
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
  switch (error_code) {
    case kNoError:
      return "";  // 正常時はメモリ割り当てを回避しパフォーマンスを向上させるためにエラーメッセージを空文字にしておく．
    case kNoConverge:
      return "Failed to converge";
    case kUndefined:
      return "Undefined value";
    case kNotUpToDate:
      return "Internal data structures not up to date with Tree or Chain";
    case kSizeMismatch:
      return "The size of the input does not match the internal state";
    case kMaxIterationExceeded:
      return "The maximum number of iterations is exceeded";
    case kOutputRange:
      return "The requested index is out of range";
    case kNotImplemented:
      return "The requested function is not yet implemented";
    case kSvdFailed:
      return "SVD failed";
    case kNotFound:
      return "Something is not found";
    case kNegativeDeltaTime:
      return "Delta time is negative";
    default:
      return "UNKNOWN ERROR";
  }
}
}  // namespace kdl
}  // namespace tobas
