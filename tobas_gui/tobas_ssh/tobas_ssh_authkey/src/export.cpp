#include "tobas_ssh_authkey/export.hpp"

#include <cstring>

namespace tobas
{
namespace ssh
{
namespace ak
{
std::expected<std::string, std::string> exportLine(const Data& src)
{
  if (src.key_type == SSH_KEYTYPE_UNKNOWN) {
    return std::unexpected("unknown key type");
  }

  if (!src.key) {
    return std::unexpected("null ssh key");
  }

  // 公開鍵を base64 で出力
  char* key_b64 = nullptr;
  if (ssh_pki_export_pubkey_base64(src.key, &key_b64) != SSH_OK || !key_b64) {
    return std::unexpected("ssh_pki_export_pubkey_base64 failed");
  }

  // 文字列を作成
  const std::string type_name(ssh_key_type_to_char(src.key_type));
  const auto line = type_name + " " + key_b64 + " " + src.comment;

  // libssh が返した文字列を解放
  ssh_string_free_char(key_b64);

  return line;
}
}  // namespace ak
}  // namespace ssh
}  // namespace tobas
