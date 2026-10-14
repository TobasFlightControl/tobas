// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>
#include <utility>
#include <vector>

namespace tobas
{
namespace str
{
/** Split a string and return a vector. */
std::vector<std::string> split(const std::string& _s, char _c);

/** Split a string at the last specified character. */
std::expected<std::pair<std::string, std::string>, std::string> rsplit(const std::string& _s, char _c);

/** Remove a specific string from the beginning. */
std::string lstrip(const std::string& _s, const std::string& _del);

/** Remove a specific string from the end. */
std::string rstrip(const std::string& _s, const std::string& _del);

/** Remove quotation marks. */
std::string stripQuates(const std::string& _s);

/** Remove leading and trailing spaces, tabs, and control characters such as newlines. */
std::string trim(const std::string& _s);

/** Split lines. */
std::vector<std::string> splitLines(const std::string& _text);

/** Remove newline characters from a string. */
std::string deleteNl(const std::string& _s);

/** Convert to lowercase. */
std::string toLower(std::string _arg);

/** Convert to uppercase. */
std::string toUpper(std::string _arg);

/** Replace a specific string in a string with another string. */
std::string replace(std::string _s, const std::string& _from, const std::string& _to);

/** Remove control characters from a string. */
std::string sanitize(const char* _s);

/**
 * Reject empty names, `.` and `..`, ASCII control characters, and reserved punctuation.
 * This does not check Windows reserved names, trailing spaces or periods, or filesystem length limits.
 */
bool isValidFileName(const std::string& _file_name);

/**
 * Check whether an email address is valid.
 *
 * @see https://www.geeksforgeeks.org/check-if-email-address-valid-or-not-in-python/
 */
bool isValidEmail(const std::string& _email);

/** Converts digits following a caret (^) into their superscript equivalent. */
std::string convertToSuperscript(const std::string& _input);

/** Convert Title Case to PascalCase. */
std::string pascalFromTitle(const std::string& _title_case);

/** Convert snake_case to PascalCase. */
std::string pascalFromSnake(const std::string& _snake_case);

/** Convert snake_case to Title Case. */
std::string titleFromSnake(const std::string& _snake_case);

/** Convert PascalCase to snake_case. */
std::string snakeFromPascal(const std::string& _pascal_case);

/** Convert Title Case to snake_case. */
std::string snakeFromTitle(const std::string& _title_case);
}  // namespace str
}  // namespace tobas
