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

::urdf::ModelInterfaceSharedPtr Parser::parseFromPath(const std::string& path)
{
  console_bridge::useOutputHandler(&oh_);

  const auto res = ::urdf::parseURDFFile(path);
  if (!res) {
    error_msg_ = oh_.message();
    oh_.clear();
  }

  console_bridge::restorePreviousOutputHandler();

  return res;
}

::urdf::ModelInterfaceSharedPtr Parser::parseFromText(const std::string& xml)
{
  console_bridge::useOutputHandler(&oh_);

  const auto res = ::urdf::parseURDF(xml);
  if (!res) {
    error_msg_ = oh_.message();
    oh_.clear();
  }

  console_bridge::restorePreviousOutputHandler();

  return res;
}

const std::string& Parser::errorMessage() const
{
  return error_msg_;
}
}  // namespace urdf
}  // namespace tobas
