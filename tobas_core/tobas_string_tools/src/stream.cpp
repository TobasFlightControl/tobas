#include "tobas_string_tools/stream.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

namespace str
{
bool readText(const std::string& path, std::string& text)
{
  std::ifstream file(path);
  if (!file) {
    std::cerr << "Failed to open file for reading: " << path << std::endl;
    return false;
  }

  std::ostringstream oss;
  oss << file.rdbuf();
  file.close();

  text = oss.str();

  return true;
}

bool writeText(const std::string& path, const std::string& text)
{
  std::ofstream file(path);
  if (!file) {
    std::cerr << "Failed to open file for writing: " << path << std::endl;
    return false;
  }

  file << text;
  file.close();

  return true;
}
}  // namespace str
