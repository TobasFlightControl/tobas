#pragma once

#include <filesystem>

#include <rapidcsv.h>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace csv
{
rapidcsv::Document load(const std::filesystem::path& path);

template <typename T>
bool getColumn(const rapidcsv::Document& doc, const std::string& name, std::vector<T>& dst)
{
  try {
    dst = doc.GetColumn<T>(name);
  }
  catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return false;
  }

  return true;
}
}  // namespace csv
}  // namespace sa
}  // namespace gui
}  // namespace tobas
