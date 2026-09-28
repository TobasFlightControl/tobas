// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf/parser.hpp"

#include <urdf_parser/urdf_parser.h>

namespace tobas
{
namespace urdf
{
Parser::Parser() : oh_(console_bridge::CONSOLE_BRIDGE_LOG_ERROR)
{
}

std::expected<::urdf::ModelInterfaceSharedPtr, std::string> Parser::parseFromPath(const std::string& path)
{
  console_bridge::useOutputHandler(&oh_);
  const auto res = ::urdf::parseURDFFile(path);
  console_bridge::restorePreviousOutputHandler();

  if (!res) {
    const auto error_msg = oh_.message();
    oh_.clear();
    return std::unexpected(error_msg);
  }

  return res;
}

std::expected<::urdf::ModelInterfaceSharedPtr, std::string> Parser::parseFromText(const std::string& xml)
{
  console_bridge::useOutputHandler(&oh_);
  const auto res = ::urdf::parseURDF(xml);
  console_bridge::restorePreviousOutputHandler();

  if (!res) {
    const auto error_msg = oh_.message();
    oh_.clear();
    return std::unexpected(error_msg);
  }

  return res;
}

}  // namespace urdf
}  // namespace tobas
