#include "tobas_gui_common/network_config.hpp"

#include <tobas_yaml_tools/core.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace cmn
{
bool NetworkConfig::load(const fs::path& path)
{
  const auto node = yaml::load(path);
  if (!node) {
    std::cerr << node.error() << std::endl;
    return false;
  }

  if (!yaml::load(kInterfaceKey, node.value(), interface)) {
    return false;
  }

  return true;
}

bool NetworkConfig::save(const fs::path& path) const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kInterfaceKey] = interface;

  if (!yaml::save(path, node)) {
    return false;
  }

  return true;
}
}  // namespace cmn
}  // namespace gui
