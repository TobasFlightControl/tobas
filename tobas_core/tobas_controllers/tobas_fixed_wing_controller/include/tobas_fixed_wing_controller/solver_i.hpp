// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree.hpp>

namespace tobas
{
namespace fixed_wing
{
class SolverI
{
protected:
  static constexpr char kErrorNotUpToDate[] = "Internal data structures not up to date with Tree";
  static constexpr char kErrorSizeMismatch[] = "The size of input doesn't match the internal state";
  static constexpr char kErrorUnknownErrorCode[] = "Unknown error code";
  static constexpr char kOutOfRange[] = "The requested index is out of range";

public:
  enum Error : int
  {
    kNoError = 0,  // No error.
    kWarn = -1,    // Warning only; continue processing.
    kError = -2,   // Serious error that should stop processing.
  };

  virtual bool updateInternalDataStructures() = 0;

  inline const int& errorCode() const;
  inline const std::string& errorMessage() const;

protected:
  int error_code_ = kNoError;
  std::string error_msg_;

  /* Update the error if the argument error code is more serious, then return the current error code. */
  inline int updateError(const SolverI& arg);
};

inline const int& SolverI::errorCode() const
{
  return error_code_;
}

inline const std::string& SolverI::errorMessage() const
{
  return error_msg_;
}

inline int SolverI::updateError(const SolverI& arg)
{
  if (arg.errorCode() < error_code_) {
    error_code_ = arg.errorCode();
    error_msg_ = arg.errorMessage();
  }
  return error_code_;
}
}  // namespace fixed_wing
}  // namespace tobas
