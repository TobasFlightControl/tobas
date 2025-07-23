#include "tobas_ros2_tools/path.hpp"

#include <filesystem>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcpputils/filesystem_helper.hpp>

#include <tobas_string_tools/core.hpp>

using namespace std;
namespace fs = filesystem;
namespace rfs = rcpputils::fs;

namespace ros2
{
fs::path resolveURI(const string& uri)
{
  static constexpr char kPackagePrefix[] = "package://";
  static constexpr char kAbsPathPrefix[] = "file://";

  if (uri.starts_with(kPackagePrefix)) {
    const auto pkg_name = str::split(str::lstrip(uri, kPackagePrefix), '/').front();
    const auto rest_of_path = str::lstrip(uri, string(kPackagePrefix) + pkg_name + '/');
    const auto pkg_path = fs::path(ament_index_cpp::get_package_share_directory(pkg_name));
    return pkg_path / rest_of_path;
  }
  else if (uri.starts_with(kAbsPathPrefix)) {
    const auto path = str::lstrip(uri, kAbsPathPrefix);
    if (path.find("$(") != string::npos) {
      throw runtime_error("Embedded xacro command is not supported.");  // TODO: $(find package_name)を置換
    }
    return fs::absolute(path);
  }
  else {
    throw runtime_error("Invalid URI: " + uri);
  }
}

int createTemporalFile(std::string& path)
{
  // システムの一時ディレクトリを取得
  const auto tmp_dir = rfs::temp_directory_path();

  // テンプレート文字列を作成．末尾6文字がXでなければならない．
  path = (tmp_dir / "tobas_temporal_file_XXXXXX").string();

  // mkstemp()はテンプレートのX部分をランダムな文字列に置き換え，一時ファイルを作成する．
  return mkstemp(path.data());
}
}  // namespace ros2
