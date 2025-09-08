#pragma once

#include <tobas_colcon_cpp/core.hpp>

namespace gui
{
namespace cmn
{
class LocalProjectBuilder
{
public:
  explicit LocalProjectBuilder();

  bool build(const std::filesystem::path& proj_path);

  const std::string& errorMessage() const;

private:
  colcon::Colcon colcon_;
};
}  // namespace cmn
}  // namespace gui
