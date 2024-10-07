#pragma once

#include <vector>
#include <string>

namespace tobas_std
{
/* 文字列をdelで区切ってvectorにして返す． */
std::vector<std::string> split(const std::string& str, const char& del);

/* 先頭の特定の文字列を削除する． */
std::string lstrip(const std::string& str, const std::string& del);

/* 末尾の特定の文字列を削除する． */
std::string rstrip(const std::string& str, const std::string& del);

/* 文字列中の改行コードを削除． */
std::string deleteNl(const std::string& str);

/* 小文字に変換． */
std::string toLower(std::string arg);

/* 大文字に変換． */
std::string toUpper(std::string arg);

/* 文字列中の特定の文字列を別の文字列に変換する． */
std::string replace(std::string str, const std::string& from, const std::string& to);

/* Converts digits following a caret (^) into their superscript equivalent. */
std::string convertToSuperscript(const std::string& input);

/* 部分文字列が含まれるかどうかを調べる． */
bool contains(const std::string& str, const std::string& sub);

/* 部分文字列が含まれるかどうかを調べる． */
bool contains(const std::string& str, const char& sub);
}  // namespace tobas_std
