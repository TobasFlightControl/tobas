#pragma once

#include <stdexcept>

namespace tobas
{
/* This may be thrown during construction of objects if errors occur. */
class ConstructException : public std::runtime_error
{
public:
  explicit ConstructException(const std::string& what_arg);
};

/* This may be thrown if unrecoverable errors occur. */
class Exception : public std::runtime_error
{
public:
  explicit Exception(const std::string& what_arg);
};
}  // namespace tobas
