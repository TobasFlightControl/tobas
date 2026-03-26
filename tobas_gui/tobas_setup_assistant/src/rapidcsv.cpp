#include "tobas_setup_assistant/rapidcsv.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace sa
{
namespace csv
{
rapidcsv::Document load(const fs::path& path)
{
  if (!fs::is_regular_file(path)) {
    throw std::runtime_error(path.string() + " does not exist.");
  }

  const rapidcsv::LabelParams label_params;

  const rapidcsv::SeparatorParams separator_params;

  const rapidcsv::ConverterParams converter_params;

  // Skip comment and blank lines
  const rapidcsv::LineReaderParams line_reader_params(true, '#', true);

  return rapidcsv::Document(path, label_params, separator_params, converter_params, line_reader_params);
}
}  // namespace csv
}  // namespace sa
}  // namespace gui
}  // namespace tobas
