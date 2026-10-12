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
std::string xmlDocumentToString(const tinyxml2::XMLDocument* _doc);

template <typename T>
std::optional<T> getAttribute(tinyxml2::XMLElement* _elem, const char* _name)
{
  T res;
  if (_elem->QueryAttribute(_name, &res) == tinyxml2::XMLError::XML_SUCCESS) {
    return res;
  }
  else {
    return std::nullopt;
  }
}

template <typename T>
void setAttribute(tinyxml2::XMLElement* _elem, const char* _name, const std::optional<T>& _value)
{
  if (_value) {
    _elem->SetAttribute(_name, *_value);
  }
}
}  // namespace xml
}  // namespace tobas
