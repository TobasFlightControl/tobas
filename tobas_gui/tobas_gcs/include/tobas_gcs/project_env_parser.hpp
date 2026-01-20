#pragma once

#include <string>

namespace gui
{
namespace gcs
{
class ProjectEnvParser
{
  static constexpr char kConfigPkgPrefix[] = "TOBAS_CONFIG_PKG=";

public:
  std::string config_pkg;

  explicit ProjectEnvParser();

  bool parseFromText(const std::string& text);

  std::string exportText() const;
};
}  // namespace gcs
}  // namespace gui
