// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/package.hpp"

#include <tinyxml2.h>

namespace fs = std::filesystem;

namespace tobas
{
namespace ros2
{
namespace
{
inline bool isLower(char _c)
{
  return _c >= 'a' && _c <= 'z';
}

inline bool isDigit(char _c)
{
  return _c >= '0' && _c <= '9';
}
}  // namespace

std::expected<fs::path, std::string> getPackagePathOf(const fs::path& _path)
{
  if (!fs::exists(_path)) {
    return std::unexpected(_path.string() + " does not exist.");
  }

  if (fs::is_directory(_path)) {
    const auto pkg_xml_path = _path / "package.xml";
    if (fs::is_regular_file(pkg_xml_path)) {
      return _path;
    }
  }

  const auto parent_path = _path.parent_path();
  if (_path == parent_path) {
    return std::unexpected("package.xml not found.");
  }

  return getPackagePathOf(parent_path);
}

std::expected<std::string, std::string> getPackageNameOf(const fs::path& _path)
{
  const auto pkg_path = getPackagePathOf(_path);
  if (!pkg_path) {
    return std::unexpected(_path.string() + " is not in a ROS package: " + pkg_path.error());
  }

  const auto pkg_xml_path = *pkg_path / "package.xml";
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(pkg_xml_path.c_str()) != tinyxml2::XML_SUCCESS) {
    return std::unexpected(doc.ErrorStr());
  }

  const auto pkg = doc.FirstChildElement("package");
  if (!pkg) {
    return std::unexpected("'package' element not found.");
  }

  const auto name = pkg->FirstChildElement("name");
  if (!name) {
    return std::unexpected("'name' element not found.");
  }

  return name->GetText();
}

std::expected<fs::path, std::string> estimateWorkspaceOf(const fs::path& _path)
{
  const auto pkg_path = getPackagePathOf(_path);
  if (!pkg_path) {
    return std::unexpected("'" + _path.string() + "' is not in a ROS package: " + pkg_path.error());
  }

  auto cur = pkg_path->parent_path();
  while (true) {
    const auto parent = cur.parent_path();
    if (cur.filename() == "src") {
      return parent;
    }
    if (parent == cur) {
      return std::unexpected("No 'src' directory found.");
    }
    cur = parent;
  }
}

bool isAlreadyBuiltAndInstalled(const fs::path& _pkg_path)
{
  // Directly under `share`.
  if (_pkg_path.parent_path().filename() != "share") {
    return false;
  }

  // Not a C++ package.
  const auto cmake_lists_path = _pkg_path / "CMakeLists.txt";
  if (fs::is_regular_file(cmake_lists_path)) {
    return false;
  }

  // Not a Python package.
  const auto setup_py_path = _pkg_path / "setup.py";
  if (fs::is_regular_file(setup_py_path)) {
    return false;
  }

  return true;
}

bool isValidPackageName(const std::string& _pkg_name)
{
  if (_pkg_name.empty()) {
    return false;
  }

  const auto& c0 = _pkg_name[0];
  if (!isLower(c0)) {
    return false;
  }

  for (size_t i = 1; i < _pkg_name.size(); ++i) {
    const auto& c = _pkg_name[i];
    if (!isLower(c) && !isDigit(c) && c != '_') {
      return false;
    }
  }

  return true;
}
}  // namespace ros2
}  // namespace tobas
