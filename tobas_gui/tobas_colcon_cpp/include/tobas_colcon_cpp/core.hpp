#pragma once

#include <expected>
#include <filesystem>
#include <string>

#include <tobas_linux/command_executor.hpp>

namespace colcon
{
class Colcon
{
public:
  explicit Colcon();

  bool build(const std::filesystem::path& pkg_path, const std::filesystem::path& ws_path);

  bool cleanWorkspace(const std::filesystem::path& ws_path);

  const std::string& errorMessage() const;

private:
  linux::CommandExecutor cmd_exec_;

  std::string error_msg_;

  static std::filesystem::path buildBase(const std::filesystem::path& ws_path);
  static std::filesystem::path installBase(const std::filesystem::path& ws_path);
  static std::filesystem::path logBase(const std::filesystem::path& ws_path);
};
}  // namespace colcon
