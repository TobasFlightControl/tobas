// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <tinyxml2.h>

#include <tobas_urdf/parser.hpp>

#include "./model.hpp"

namespace tobas
{
namespace uadf
{
class Parser
{
public:
  explicit Parser();

  std::expected<void, std::string> parseFromXml(const tinyxml2::XMLDocument* uadf_doc, Model& uadf_model);
  std::expected<void, std::string> parseFromText(const std::string& uadf_text, Model& uadf_model);
  std::expected<void, std::string> parseFromPath(const std::string& uadf_path, Model& uadf_model);

private:
  urdf::Parser urdf_parser_;
};
}  // namespace uadf
}  // namespace tobas
