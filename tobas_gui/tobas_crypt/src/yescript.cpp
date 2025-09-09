#include "tobas_crypt/yescrypt.hpp"

#include <crypt.h>

#include <iostream>

#include <tobas_linux/error.hpp>

using namespace std;

namespace tobas
{
namespace crypt
{
Yescrypt::Yescrypt()
{
}

string Yescrypt::createSalt() const
{
  char salt[CRYPT_GENSALT_OUTPUT_SIZE]{};
  if (!crypt_gensalt_rn("$y$", 0, nullptr, 0, salt, sizeof(salt))) {
    cerr << "crypt_gensalt_rn failed: " << linux::strError() << endl;
    return {};
  }
  return string(salt);
}
}  // namespace crypt
}  // namespace tobas
