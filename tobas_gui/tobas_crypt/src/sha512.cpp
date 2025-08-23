#include "tobas_crypt/sha512.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <vector>

#include <tobas_linux/error.hpp>

using namespace std;

namespace tobas
{
namespace crypt
{
Sha512::Sha512(int rounds) : rounds_(rounds)
{
}

string Sha512::createSalt() const
{
  const auto fd = ::open(kUrandomPath, O_RDONLY);
  if (fd < 0) {
    cerr << "Failed to open " << kUrandomPath << ": " << linux::strError() << endl;
    return {};
  }

  vector<unsigned char> buf(kLength);
  const auto n = ::read(fd, buf.data(), buf.size());
  ::close(fd);

  if (n != static_cast<ssize_t>(buf.size())) {
    cerr << "Failed to read urandom." << endl;
    return {};
  }

  constexpr char tbl[] = "./0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  string salt;
  salt.resize(kLength);
  for (size_t i = 0; i < kLength; ++i) {
    salt[i] = tbl[buf[i] & 63];  // 0..63 -> 64種
  }

  return "$6$rounds=" + to_string(rounds_) + "$" + salt;
}
}  // namespace crypt
}  // namespace tobas
