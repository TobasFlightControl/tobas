#pragma once

#include <string>
#include <vector>

namespace tobas_std
{
/* 文字列をdelで区切ってvectorにして返す． */
std::vector<std::string> split(const std::string& str, const char& del);

/* 文字列を最後の指定文字で区切る． */
std::pair<std::string, std::string> rsplit(const std::string& str, const char& c);

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

/* 部分文字列が含まれるかどうかを調べる． */
bool contains(const std::string& str, const std::string& sub);

/* 部分文字列が含まれるかどうかを調べる． */
bool contains(const std::string& str, const char& sub);

/**
 * @brief Emailアドレスが有効かどうかを判定する．
 * cf. https://www.geeksforgeeks.org/check-if-email-address-valid-or-not-in-python/
 */
bool isValidEmail(const std::string& email);

/* Converts digits following a caret (^) into their superscript equivalent. */
std::string convertToSuperscript(const std::string& input);

/* Title CaseをPascalCaseに変換する． */
std::string pascalFromTitle(const std::string& title_case);

/* snake_caseをPascalCaseに変換する． */
std::string pascalFromSnake(const std::string& snake_case);

/* snake_caseをTitle Caseに変換する． */
std::string titleFromSnake(const std::string& snake_case);

/* PascalCaseをsnake_caseに変換する． */
std::string snakeFromPascal(const std::string& pascal_case);

/* Title Caseをsnake_caseに変換する． */
std::string snakeFromTitle(const std::string& title_case);
}  // namespace tobas_std
