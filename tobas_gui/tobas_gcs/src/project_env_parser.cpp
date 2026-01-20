#include "tobas_gcs/project_env_parser.hpp"

#include <sstream>

#include <tobas_string_tools/core.hpp>

namespace gui
{
namespace gcs
{
ProjectEnvParser::ProjectEnvParser()
{
}

bool ProjectEnvParser::parseFromText(const std::string& text)
{
  const auto lines = str::splitLines(text);

  for (auto line : lines) {
    // Trim whitespaces
    line = str::trim(line);

    // Skip blank lines and comments
    if (line.empty() || line.starts_with('#')) {
      continue;
    }

    // Configuration package name
    if (line.starts_with(kConfigPkgPrefix)) {
      config_pkg = line.substr(sizeof(kConfigPkgPrefix) - 1);
      continue;
    }
  }

  return true;
}

std::string ProjectEnvParser::exportText() const
{
  std::ostringstream oss;

  oss << kConfigPkgPrefix << config_pkg << std::endl;

  return oss.str();
}
}  // namespace gcs
}  // namespace gui
