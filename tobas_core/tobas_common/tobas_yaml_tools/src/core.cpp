#include "tobas_yaml_tools/core.hpp"

#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

namespace yaml
{
std::string dump(const YAML::Node& node)
{
  std::ostringstream res;
  YAML::Emitter emitter(res);
  emitter << node;
  res << std::endl;
  return res.str();
}

std::expected<YAML::Node, std::string> load(const fs::path& path)
{
  if (!fs::exists(path)) {
    return std::unexpected(path.string() + " does not exist.");
  }

  try {
    return YAML::LoadFile(path);
  }
  catch (const std::exception& e) {
    return std::unexpected("Failed to load " + path.string() + ": " + e.what());
  }
}

bool save(const fs::path& path, const YAML::Node& node)
{
  std::ofstream fout(path);
  if (!fout.is_open()) {
    std::cerr << "Failed to open \"" << path << "\" for writing." << std::endl;
    return false;
  }

  fout << dump(node);
  fout.close();

  return true;
}
}  // namespace yaml
