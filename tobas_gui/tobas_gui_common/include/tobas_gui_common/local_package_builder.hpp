#pragma once

#include <filesystem>

#include <tobas_linux/command_executor.hpp>

namespace gui
{
namespace common
{
class LocalPackageBuilder
{
public:
  explicit LocalPackageBuilder();

  bool build(const std::filesystem::path& tbs_path);

  const std::string& getOutput() const;

private:
  linux::CommandExecutor command_executor_;
};
}  // namespace common
}  // namespace gui
