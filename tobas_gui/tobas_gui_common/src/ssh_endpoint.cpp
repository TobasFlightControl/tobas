#include "tobas_gui_common/ssh_endpoint.hpp"

#include <tobas_yaml_tools/core.hpp>

#include "tobas_gui_common/project_paths.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace common
{
bool SshEndpoint::load(const fs::path& path)
{
  const auto node = yaml::load(path);
  if (!node) {
    std::cerr << node.error() << std::endl;
    return false;
  }

  if (!yaml::load(kHostKey, node.value(), host)) {
    return false;
  }
  if (!yaml::load(kUserKey, node.value(), user)) {
    return false;
  }

  return true;
}

bool SshEndpoint::save(const fs::path& path) const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kHostKey] = host;
  node[kUserKey] = user;

  if (!yaml::save(path, node)) {
    return false;
  }

  return true;
}
}  // namespace common
}  // namespace gui
