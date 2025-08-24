#pragma once

#include <expected>

#include "./data.hpp"

namespace tobas
{
namespace sak
{
std::expected<std::string, std::string> exportLine(const Data& src);
}  // namespace sak
}  // namespace tobas
