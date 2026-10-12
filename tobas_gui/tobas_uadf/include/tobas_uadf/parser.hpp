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

  std::expected<Model, std::string> parseFromXml(const tinyxml2::XMLDocument* uadf_doc);
  std::expected<Model, std::string> parseFromText(const std::string& uadf_text);
  std::expected<Model, std::string> parseFromPath(const std::string& uadf_path);

private:
  urdf::Parser urdf_parser_;
};
}  // namespace uadf
}  // namespace tobas
