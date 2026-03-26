#include "tobas_urdf/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_string_tools/core.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace urdf
{
fs::path resolveURI(const std::string& uri)
{
  static constexpr char kPackagePrefix[] = "package://";
  static constexpr char kAbsPathPrefix[] = "file://";

  if (uri.starts_with(kPackagePrefix)) {
    const auto pkg_name = str::split(str::lstrip(uri, kPackagePrefix), '/').front();
    const auto rest_of_path = str::lstrip(uri, std::string(kPackagePrefix) + pkg_name + '/');
    const fs::path pkg_path(ament_index_cpp::get_package_share_directory(pkg_name));
    return pkg_path / rest_of_path;
  }
  else if (uri.starts_with(kAbsPathPrefix)) {
    const auto path = str::lstrip(uri, kAbsPathPrefix);
    if (path.find("$(") != std::string::npos) {
      throw std::runtime_error("Embedded xacro command is not supported.");  // TODO: $(find package_name)を置換
    }
    return fs::absolute(path);
  }
  else {
    throw std::runtime_error("Invalid URI: " + uri);
  }
}
}  // namespace urdf
}  // namespace tobas
