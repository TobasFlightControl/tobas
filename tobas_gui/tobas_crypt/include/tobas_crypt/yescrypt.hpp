#pragma once

#include "./base.hpp"

namespace tobas
{
namespace crypt
{
class Yescrypt : public Crypt
{
public:
  explicit Yescrypt();

private:
  std::string createSalt() const override;
};
}  // namespace crypt
}  // namespace tobas
