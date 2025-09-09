#pragma once

#include <expected>

#include "./data.hpp"

namespace tobas
{
namespace ssh
{
namespace ak
{
/* SSH公開鍵を表示用に文字列にする． */
std::expected<std::string, std::string> prettify(const Data& src);
}  // namespace ak
}  // namespace ssh
}  // namespace tobas
