#include "tobas_ros2_tools/package.hpp"

#include <tinyxml2.h>

namespace fs = std::filesystem;

namespace ros2
{
std::expected<fs::path, std::string> getPackagePathOf(const fs::path& path)
{
  if (!fs::exists(path)) {
    return std::unexpected(path.string() + " does not exist.");
  }

  if (fs::is_directory(path)) {
    const auto pkg_xml_path = path / "package.xml";
    if (fs::is_regular_file(pkg_xml_path)) {
      return path;
    }
  }

  const auto parent_path = path.parent_path();
  if (path == parent_path) {
    return std::unexpected("package.xml not found.");
  }

  return getPackagePathOf(parent_path);
}

std::expected<std::string, std::string> getPackageNameOf(const fs::path& path)
{
  const auto pkg_path = getPackagePathOf(path);
  if (!pkg_path) {
    return std::unexpected(path.string() + " is not in a ROS package: " + pkg_path.error());
  }

  const auto pkg_xml_path = pkg_path.value() / "package.xml";
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(pkg_xml_path.c_str()) != tinyxml2::XML_SUCCESS) {
    return std::unexpected(doc.ErrorStr());
  }

  const auto pkg = doc.FirstChildElement("package");
  if (!pkg) {
    return std::unexpected("\"package\" element not found.");
  }

  const auto name = pkg->FirstChildElement("name");
  if (!name) {
    return std::unexpected("\"name\" element not found.");
  }

  return name->GetText();
}

std::expected<fs::path, std::string> estimateWorkspaceOf(const fs::path& path)
{
  const auto pkg_path = getPackagePathOf(path);
  if (!pkg_path) {
    return std::unexpected(path.string() + " is not in a ROS package: " + pkg_path.error());
  }

  auto cur = pkg_path.value().parent_path();
  while (true) {
    const auto parent = cur.parent_path();
    if (cur.filename() == "src") {
      return parent;
    }
    if (parent == cur) {
      return std::unexpected("No \"src\" directory found.");
    }
    cur = parent;
  }
}
}  // namespace ros2
