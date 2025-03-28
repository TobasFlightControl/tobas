#pragma once

#include <stdexcept>

/** \brief Main namespace for Tobas */
namespace tobas
{
/** \brief This may be thrown during construction of objects if errors occur */
class ConstructException : public std::runtime_error
{
public:
  explicit ConstructException(const std::string& what_arg);
};

/** \brief This may be thrown if unrecoverable errors occur */
class Exception : public std::runtime_error
{
public:
  explicit Exception(const std::string& what_arg);
};
}  // namespace tobas
