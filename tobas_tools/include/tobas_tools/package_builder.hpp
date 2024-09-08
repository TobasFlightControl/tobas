#pragma once

#include <filesystem>

#include <tobas_linux/command_executor.hpp>

namespace tobas
{
class PackageBuilder
{
public:
  explicit PackageBuilder();

  bool build(const std::filesystem::path& tbs_path);

  const std::string& getOutput() const;

private:
  linux::CommandExecutor command_executor_;
};
}  // namespace tobas
