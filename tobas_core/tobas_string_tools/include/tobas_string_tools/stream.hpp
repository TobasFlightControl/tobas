#pragma once

#include <string>

namespace str
{
bool readText(const std::string& path, std::string& text);

bool writeText(const std::string& path, const std::string& text);
}  // namespace str
