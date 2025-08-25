#pragma once

#include "./base.hpp"

namespace tobas
{
namespace crypt
{
class Sha512 : public Crypt
{
  static constexpr char kUrandomPath[] = "/dev/urandom";
  static constexpr size_t kLength = 16;

public:
  explicit Sha512(int rounds = 500000);

private:
  const int rounds_;

  std::string createSalt() const override;
};
}  // namespace crypt
}  // namespace tobas
