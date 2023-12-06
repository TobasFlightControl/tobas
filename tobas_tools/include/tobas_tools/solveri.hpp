#pragma once

#include <string>

namespace tobas
{
class SolverI
{
protected:
  static constexpr char kErrorSizeMismatch[] = "The size of input doesn't match the internal state";

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
