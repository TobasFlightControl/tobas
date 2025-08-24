#pragma once

#include <expected>

#include "./data.hpp"

namespace tobas
{
namespace sak
{
std::expected<Data, std::string> parseLine(const std::string& line);
}  // namespace sak
}  // namespace tobas
