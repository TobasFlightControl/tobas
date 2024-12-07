#pragma once

#include <string>

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
  enum error_t : int
  {
    E_NOERROR = 0,                   // No error
    E_NO_CONVERGE = -1,              // Failed to converge
    E_UNDEFINED = -2,                // Undefined value (e.g. computed a NAN, or tan(90 degrees) )
    E_NOT_UP_TO_DATE = -3,           // Chain size changed
    E_SIZE_MISMATCH = -4,            // Input size does not match internal state
    E_MAX_ITERATIONS_EXCEEDED = -5,  // Maximum number of iterations exceeded
    E_OUT_OF_RANGE = -6,             // Requested index out of range
    E_NOT_IMPLEMENTED = -7,          // Not yet implemented
    E_SVD_FAILED = -8,               // Internal SVD calculation failed
    E_QP_FAILED = -9,                // Internal QP calculation failed
    E_NOT_FOUND = -10,               // Something is not found
    E_NEGATIVE_DELTA_TIME = -11,     // Negative delta time
    E_UNKNOWN = -99,                 // Unknown error
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
  int error_code_ = E_NOERROR;  // Latest error code
  std::string error_msg_;       // Latest error string

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
