#pragma once

#include <string>

namespace gui
{
namespace core
{
class ConfigurationEnvParser
{
  static constexpr char kConfigPkgPrefix[] = "TOBAS_CONFIG_PKG=";

public:
  std::string config_pkg;

  explicit ConfigurationEnvParser();

  bool parseFromText(const std::string& text);

  std::string exportText() const;

private:
};
}  // namespace core
}  // namespace gui
