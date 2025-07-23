#pragma once

#include <string>

#include <tobas_linux/command_executor.hpp>

namespace gui
{
namespace sa
{
class XacroParser
{
public:
  explicit XacroParser();

  bool parseFromPath(const std::string& xacro_path, std::string& urdf_text);
  bool parseFromText(const std::string& xacro_text, std::string& urdf_text);

  const std::string& getOutput() const;

private:
  linux::CommandExecutor command_executor_;
};
}  // namespace sa
}  // namespace gui
