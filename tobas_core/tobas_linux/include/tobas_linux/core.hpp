#pragma once

#include <string>

namespace linux
{
/* ユーザ名を取得する． */
std::string userName();

/* ホームディレクトリを取得する． */
std::string homeDir();

/* ホームディレクトリを絶対パスに変換する． */
std::string expandUser(const std::string& path);

/* プログラムがRoot権限で実行されている場合にTrueを返す． */
bool isSuperUser();

/* コマンドラインの実行結果を取得する． */
std::string executeCommand(const char* command);

/* Gitのユーザ名を返す． */
std::string getGitUserName();

/* Gitのメールアドレスを返す． */
std::string getGitUserEmail();
}  // namespace linux
