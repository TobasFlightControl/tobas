#include "tobas_setup_assistant/template_generator.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sa
{
TemplateGenerator::TemplateGenerator(const fs::path& tpl_dir) : tpl_dir_(tpl_dir)
{
}

void TemplateGenerator::generate(const inja::json& data, const fs::path& rel_path, const fs::path& out_dir, bool overwrite)
{
  const auto tpl_path = tpl_dir_ / rel_path;
  const auto out_path = out_dir / rel_path.stem();

  if (!overwrite && fs::exists(out_path)) {
    return;
  }

  const auto temp = env_.parse_template(tpl_path);
  env_.write(temp, data, out_path);
}
}  // namespace sa
}  // namespace gui
