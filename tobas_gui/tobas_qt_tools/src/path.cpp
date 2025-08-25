#include "tobas_qt_tools/path.hpp"

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace qt
{
QString getBaseName(const QString& path)
{
  return path.left(path.lastIndexOf('.'));
}

QString getResourcePath()
{
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory("tobas_qt_tools"));
  const auto resource_path = pkg_path / "resources";
  return QString::fromStdString(resource_path);
}
}  // namespace qt
