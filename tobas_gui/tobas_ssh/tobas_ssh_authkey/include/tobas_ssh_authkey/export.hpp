#pragma once

#include <expected>

#include "./data.hpp"

namespace tobas
{
namespace ssh
{
namespace ak
{
std::expected<std::string, std::string> exportLine(const Data& src);
}  // namespace ak
}  // namespace ssh
}  // namespace tobas
