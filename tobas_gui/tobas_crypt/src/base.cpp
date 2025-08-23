#include "tobas_crypt/base.hpp"

#include <crypt.h>

#include <cstring>
#include <iostream>

#include <tobas_linux/error.hpp>

using namespace std;

namespace tobas
{
namespace crypt
{
string Crypt::crypt(const string& password) const
{
  struct crypt_data data;
  memset(&data, 0, sizeof(data));

  const auto salt = createSalt();
  const auto out = ::crypt_r(password.c_str(), salt.c_str(), &data);
  if (!out) {
    cerr << "crypt_r failed: " << linux::strError() << endl;
    return {};
  }

  return string(out);
}
}  // namespace crypt
}  // namespace tobas
