#pragma once

#include <string>

namespace tobas_std
{
/* プログラムがRoot権限で実行されている場合にTrueを返す． */
bool isSuperUser();

/* コマンドラインの実行結果を取得する． */
std::string exec_command(const char* command);
}  // namespace tobas_std
