#pragma once

#include <string>
#include <vector>

#include <libssh/libssh.h>

namespace tobas
{
namespace sak
{
struct Data
{
  ssh_keytypes_e key_type = SSH_KEYTYPE_UNKNOWN;
  ssh_key key = nullptr;
  std::string comment = "";
};
}  // namespace sak
}  // namespace tobas
