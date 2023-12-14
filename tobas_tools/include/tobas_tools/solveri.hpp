#pragma once

#include <string>

namespace tobas
{
class SolverI
{
protected:
  static constexpr char kErrorNotUpToDate[] = "Internal data structures not up to date with Tree";
  static constexpr char kErrorSizeMismatch[] = "The size of input doesn't match the internal state";
  static constexpr char kOutOfRange[] = "The requested index is out of range";

public:
  virtual void updateInternalDataStructures() = 0;

  virtual const std::string& errorMessage() const
  {
    return error_msg_;
  }

protected:
  std::string error_msg_;
};
}  // namespace tobas
