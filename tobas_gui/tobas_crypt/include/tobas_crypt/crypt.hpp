#pragma once

#include "./base.hpp"

namespace tobas
{
namespace crypt
{
/* ユーザーのパスワードを更新する． */
bool setShadowPassword(
  const std::string& _shadow_path,
  const std::string& _username,
  const std::string& _new_password,
  const Crypt& _crypt);
}  // namespace crypt
}  // namespace tobas
