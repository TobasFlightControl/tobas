#pragma once

#include <string>

namespace tobas
{
namespace crypt
{
class Crypt
{
public:
  std::string crypt(const std::string& password) const;

private:
  virtual std::string createSalt() const = 0;
};
}  // namespace crypt
}  // namespace tobas
