#include "tobas_yaml_tools/core.hpp"

#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

namespace yaml
{
std::expected<std::string, std::string> dump(const YAML::Node& node, size_t precision)
{
  std::ostringstream res;
  YAML::Emitter emitter(res);

  if (!emitter.SetFloatPrecision(precision)) {
    return std::unexpected("Failed to set the yaml float precision to " + std::to_string(precision) + ".");
  }
  if (!emitter.SetDoublePrecision(precision)) {
    return std::unexpected("Failed to set the yaml double precision to " + std::to_string(precision) + ".");
  }

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

bool save(const fs::path& path, const YAML::Node& node, size_t precision)
{
  const auto text = dump(node, precision);
  if (!text) {
    std::cerr << text.error() << std::endl;
    return false;
  }

  std::ofstream fout(path);
  if (!fout.is_open()) {
    std::cerr << "Failed to open \"" << path << "\" for writing." << std::endl;
    return false;
  }

  fout << text.value();
  fout.close();

  return true;
}
}  // namespace yaml
