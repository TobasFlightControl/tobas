#pragma once

#include <filesystem>

namespace gui
{
namespace cmn
{
class SshConfig
{
  static constexpr char kHostKey[] = "host";
  static constexpr char kUserKey[] = "user";

public:
  std::string host;
  std::string user;

  bool load(const std::filesystem::path& path);
  bool save(const std::filesystem::path& path) const;
};
}  // namespace cmn
}  // namespace gui
