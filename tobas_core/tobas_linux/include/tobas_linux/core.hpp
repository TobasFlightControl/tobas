#pragma once

#include <filesystem>

namespace linux
{
/* ユーザ名を取得する． */
std::string userName();

/* ホームディレクトリを取得する． */
std::filesystem::path homeDir();

/* ホームディレクトリを絶対パスに変換する． */
std::filesystem::path expandUser(const std::string& path);

/* プログラムがRoot権限で実行されている場合にTrueを返す． */
bool isSuperUser();
}  // namespace linux
