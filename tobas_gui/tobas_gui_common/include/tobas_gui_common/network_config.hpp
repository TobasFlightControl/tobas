#pragma once

#include <filesystem>

namespace tobas
{
namespace gui
{
namespace cmn
{
class NetworkConfig
{
  static constexpr char kInterfaceKey[] = "interface";

public:
  std::string interface;

  bool load(const std::filesystem::path& path);
  bool save(const std::filesystem::path& path) const;
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
