// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_xml_tools/core.hpp"

namespace tobas::xml
{
std::string xmlDocumentToString(const tinyxml2::XMLDocument* _doc)
{
  tinyxml2::XMLPrinter printer;
  _doc->Print(&printer);
  return printer.CStr();
}
}  // namespace tobas::xml
