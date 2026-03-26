#pragma once

#include <string>

namespace tobas
{
namespace kdl
{
/**
 * Solver interface supporting storage and description of the latest error.
 *
 * Error codes: Zero (0) indicates no error, positive error codes indicate more
 * of a warning (e.g. a degraded solution, but motion can continue), and
 * negative error codes indicate failure (e.g. a singularity, and motion
 * can not continue).
 *
 * Error codes between -99 and +99 (inclusive) are reserved for system-wide
 * error codes. Derived classes should use values > +100, and < -100.
 */
class SolverI
{
public:
  enum Error : int
  {
    kNoError = 0,                // No error
    kNoConverge = -1,            // Failed to converge
    kUndefined = -2,             // Undefined value (e.g. computed a NAN, or tan(90 degrees) )
    kNotUpToDate = -3,           // Chain size changed
    kSizeMismatch = -4,          // Input size does not match internal state
    kMaxIterationExceeded = -5,  // Maximum number of iterations exceeded
    kOutputRange = -6,           // Requested index out of range
    kNotImplemented = -7,        // Not yet implemented
    kSvdFailed = -8,             // Internal SVD calculation failed
    kQpFailed = -9,              // Internal QP calculation failed
    kNotFound = -10,             // Something is not found
    kNegativeDeltaTime = -11,    // Negative delta time
    kUnknown = -99,              // Unknown error
  };

  /**
   * Update the internal data structures. This is required if the number
   * of segments or number of joints of a chain/tree have changed.
   * This provides a single point of contact for solver memory allocations.
   */
  virtual bool updateInternalDataStructures() = 0;

  inline const int& errorCode() const;
  inline const std::string& errorMessage() const;

protected:
  int error_code_ = kNoError;  // Latest error code
  std::string error_msg_;      // Latest error string

  int copyError(const SolverI& arg);
  int setDefaultError(const int& error_code);
  virtual std::string defaultErrorMessage(const int& error_code) const;
};

inline const int& SolverI::errorCode() const
{
  return error_code_;
}

inline const std::string& SolverI::errorMessage() const
{
  return error_msg_;
}
}  // namespace kdl
}  // namespace tobas
