// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_string_tools/core.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace urdf
{
std::expected<fs::path, std::string> resolveUri(const std::string& uri)
{
  constexpr char kPackagePrefix[] = "package://";
  constexpr char kAbsPathPrefix[] = "file://";

  if (uri.starts_with(kPackagePrefix)) {
    const auto pkg_name = str::split(str::lstrip(uri, kPackagePrefix), '/').front();
    const auto rest_of_path = str::lstrip(uri, std::string(kPackagePrefix) + pkg_name + '/');
    const fs::path pkg_path(ament_index_cpp::get_package_share_directory(pkg_name));
    return pkg_path / rest_of_path;
  }
  else if (uri.starts_with(kAbsPathPrefix)) {
    const auto path = str::lstrip(uri, kAbsPathPrefix);
    if (path.find("$(") != std::string::npos) {
      return std::unexpected("Embedded xacro command is not supported.");  // TODO: Replace `$(find package_name)`.
    }
    return fs::absolute(path);
  }
  else {
    return std::unexpected("Invalid URI.");
  }
}
}  // namespace urdf
}  // namespace tobas
