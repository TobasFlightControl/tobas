#pragma once

#include <expected>
#include <filesystem>
#include <vector>

#include "./data.hpp"

namespace tobas
{
namespace sak
{
std::expected<Data, std::string> parseLine(const std::string& line);

std::expected<std::vector<Data>, std::string> parseFile(const std::filesystem::path& path);
}  // namespace sak
}  // namespace tobas
