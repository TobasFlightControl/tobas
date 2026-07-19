// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>
#include <string>

#include <tinyxml2.h>

namespace tobas
{
namespace xml
{
std::string xmlDocumentToString(const tinyxml2::XMLDocument* doc);

template <typename T>
std::optional<T> getAttribute(tinyxml2::XMLElement* elem, const char* name)
{
  T res;
  if (elem->QueryAttribute(name, &res) == tinyxml2::XMLError::XML_SUCCESS) {
    return res;
  }
  else {
    return std::nullopt;
  }
}

template <typename T>
void setAttribute(tinyxml2::XMLElement* elem, const char* name, const std::optional<T>& value)
{
  if (value) {
    elem->SetAttribute(name, value.value());
  }
}
}  // namespace xml
}  // namespace tobas
