#pragma once

#include <string>

namespace path
{
/* TODO: C++20が使えるようになったら削除 */
bool starts_with(const std::string& str, const std::string& prefix);

/* TODO: C++20が使えるようになったら削除 */
bool ends_with(const std::string& str, const std::string& suffix);
}  // namespace path
