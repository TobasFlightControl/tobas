// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/xacro_parser.hpp"

#include <fstream>
#include <iostream>

#include <tobas_ros2_tools/path.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
XacroParser::XacroParser()
{
}

bool XacroParser::parseFromPath(const std::string& xacro_path, std::string& urdf_text)
{
  // Create a temporary URDF path.
  std::string tmp_urdf_path;
  if (ros2::createTemporalFile(tmp_urdf_path) < 0) {
    std::cerr << "Failed to create a temporal URDF path." << std::endl;
    return false;
  }

  // Expand XACRO and create a URDF.
  const auto command = "xacro " + xacro_path + " > " + tmp_urdf_path;
  if (!command_executor_.execute(command)) {
    std::cerr << "Failed to convert XACRO to URDF." << std::endl;
    return false;
  }
  std::cout << "Temporary URDF is created: " << tmp_urdf_path << std::endl;

  // Read the created URDF.
  std::ifstream file(tmp_urdf_path);
  if (!file) {
    std::cerr << "Failed to open file: " << tmp_urdf_path << std::endl;
    return false;
  }
  urdf_text.clear();
  urdf_text.append((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  return true;
}

bool XacroParser::parseFromText(const std::string& xacro_text, std::string& urdf_text)
{
  // Create a temporary XACRO path.
  std::string tmp_xacro_path;
  const auto fd = ros2::createTemporalFile(tmp_xacro_path);
  if (fd < 0) {
    std::cerr << "Failed to create a temporal XACRO path." << std::endl;
    return false;
  }

  // Open the temporary file.
  const auto fp = ::fdopen(fd, "wb");
  if (!fp) {
    std::cerr << "Failed to open " << tmp_xacro_path << "." << std::endl;
    return false;
  }

  // Write XACRO contents to the temporary file.
  const auto xacro_size = xacro_text.size();
  const auto n = ::fwrite(xacro_text.data(), 1, xacro_size, fp);
  if (n != xacro_size) {
    std::cerr << "Tried to write " << xacro_size << " bytes, but only " << n << " bytes were written." << std::endl;
    return false;
  }

  // Close the temporary file.
  if (::fclose(fp) < 0) {
    std::cerr << "Failed to close " << tmp_xacro_path << "." << std::endl;
    return false;
  }

  // Generate a URDF from the saved XACRO.
  return parseFromPath(tmp_xacro_path, urdf_text);
}

const std::string& XacroParser::getOutput() const
{
  return command_executor_.getOutput();
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
