#pragma once

#include "./command_executor.hpp"

namespace tobas
{
namespace linux
{
class GitHandler
{
public:
  explicit GitHandler();

  /* ユーザ名を返す． */
  std::string getUserName();

  /* メールアドレスを返す． */
  std::string getUserEmail();

private:
  CommandExecutor command_executor_;
};
}  // namespace linux
}  // namespace tobas
