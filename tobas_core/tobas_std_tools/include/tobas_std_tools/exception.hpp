#pragma once

#include <exception>
#include <string>

namespace tobas_std
{
class NotImplementedError : public std::exception
{
public:
  NotImplementedError(const std::string& msg = "");

  virtual const char* what() const noexcept;

private:
  std::string msg_;
};
}  // namespace tobas_std
