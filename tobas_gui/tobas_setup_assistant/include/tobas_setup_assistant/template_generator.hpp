#pragma once

#include <filesystem>

#include <inja/inja.hpp>

namespace gui
{
namespace sa
{
class TemplateGenerator
{
public:
  explicit TemplateGenerator(const std::filesystem::path& tpl_dir);

  void generate(
    const inja::json& data,
    const std::filesystem::path& rel_path,
    const std::filesystem::path& out_dir,
    bool overwrite = true);

private:
  const std::filesystem::path tpl_dir_;

  inja::Environment env_;
};
}  // namespace sa
}  // namespace gui
