#include <filesystem>
#include <stdexcept>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_std_tools/string.hpp>

#include "../include/tobas_ros2_tools/path.hpp"

using namespace std;
namespace fs = filesystem;

namespace ros2
{
fs::path resolveURI(const string& uri)
{
  static constexpr char kPackagePrefix[] = "package://";
  static constexpr char kAbsPathPrefix[] = "file://";

  if (uri.starts_with(kPackagePrefix))
  {
    const auto pkg_name = tobas_std::split(tobas_std::lstrip(uri, kPackagePrefix), '/').at(0);
    const auto rest_of_path = tobas_std::lstrip(uri, string(kPackagePrefix) + pkg_name + '/');
    const auto pkg_path = fs::path(ament_index_cpp::get_package_share_directory(pkg_name));
    return pkg_path / rest_of_path;
  }
  else if (uri.starts_with(kAbsPathPrefix))
  {
    const auto path = tobas_std::lstrip(uri, kAbsPathPrefix);
    if (path.find("$(") != string::npos)
      throw runtime_error("Embedded xacro command is not supported.");  // TODO: $(find package_name)を置換
    return fs::absolute(path);
  }
  else
  {
    throw runtime_error("Invalid URI: " + uri);
  }
}
}  // namespace ros2
