#pragma once

#include <vector>
#include <string>

namespace tobas_std
{
/* 文字列をdelで区切ってvectorにして返す． */
std::vector<std::string> split(const std::string& str, const char& del);

/* 文字列中の改行コードを削除． */
std::string deleteNl(const std::string& str);

/* 小文字に変換． */
std::string toLower(std::string arg);

/* 大文字に変換． */
std::string toUpper(std::string arg);

/* 部分文字列が含まれるかどうかを調べる． */
bool contains(const std::string& str, const std::string& sub);

/* 部分文字列が含まれるかどうかを調べる． */
bool contains(const std::string& str, const char& sub);
}  // namespace tobas_std
