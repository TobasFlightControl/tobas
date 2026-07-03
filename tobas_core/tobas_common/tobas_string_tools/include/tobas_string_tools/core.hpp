// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>
#include <vector>

namespace tobas
{
namespace str
{
/* Split a string and return a vector. */
std::vector<std::string> split(const std::string& s, const char& c);

/* Split a string at the last specified character. */
std::pair<std::string, std::string> rsplit(const std::string& s, const char& c);

/* Remove a specific string from the beginning. */
std::string lstrip(const std::string& s, const std::string& del);

/* Remove a specific string from the end. */
std::string rstrip(const std::string& s, const std::string& del);

/* Remove quotation marks. */
std::string stripQuates(const std::string& s);

/* Remove leading and trailing spaces, tabs, and control characters such as newlines. */
std::string trim(const std::string& s);

/* Split lines. */
std::vector<std::string> splitLines(const std::string& text);

/* Remove newline characters from a string. */
std::string deleteNl(const std::string& s);

/* Convert to lowercase. */
std::string toLower(std::string arg);

/* Convert to uppercase. */
std::string toUpper(std::string arg);

/* Replace a specific string in a string with another string. */
std::string replace(std::string s, const std::string& from, const std::string& to);

/* Remove control characters from a string. */
std::string sanitize(const char* s);

/* Check whether a substring is included. */
bool contains(const std::string& s, const std::string& sub);

/* Check whether a character is included. */
bool contains(const std::string& s, const char& sub);

/* Check whether a file name contains no invalid strings. */
bool isValidFileName(const std::string& file_name);

/**
 * @brief Check whether an email address is valid.
 * cf. https://www.geeksforgeeks.org/check-if-email-address-valid-or-not-in-python/
 */
bool isValidEmail(const std::string& email);

/* Converts digits following a caret (^) into their superscript equivalent. */
std::string convertToSuperscript(const std::string& input);

/* Convert Title Case to PascalCase. */
std::string pascalFromTitle(const std::string& title_case);

/* Convert snake_case to PascalCase. */
std::string pascalFromSnake(const std::string& snake_case);

/* Convert snake_case to Title Case. */
std::string titleFromSnake(const std::string& snake_case);

/* Convert PascalCase to snake_case. */
std::string snakeFromPascal(const std::string& pascal_case);

/* Convert Title Case to snake_case. */
std::string snakeFromTitle(const std::string& title_case);
}  // namespace str
}  // namespace tobas
