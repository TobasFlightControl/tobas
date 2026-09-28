// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

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

  bool parseFromXml(const tinyxml2::XMLDocument* uadf_doc, Model& uadf_model);
  bool parseFromText(const std::string& uadf_text, Model& uadf_model);
  bool parseFromPath(const std::string& uadf_path, Model& uadf_model);

  const std::string& errorMessage() const;

private:
  std::string error_msg_;

  urdf::Parser urdf_parser_;
};
}  // namespace uadf
}  // namespace tobas
