#pragma once

#include <string>
#include <vector>

namespace linux
{
/* SHA-512 crypt のハッシュを生成する． */
std::string crypt_sha512(const std::string& password, int rounds = 500000);

/* yescrypt のハッシュを生成する． */
std::string crypt_yescrypt(const std::string& password);

/* ユーザーのパスワードを更新する． */
bool setShadowPassword(const std::string& shadow_path, const std::string& username, const std::string& new_password);
}  // namespace linux
