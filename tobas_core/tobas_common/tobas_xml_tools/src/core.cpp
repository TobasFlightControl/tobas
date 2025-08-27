#include "tobas_xml_tools/core.hpp"

namespace xml
{
std::string xmlDocumentToString(const tinyxml2::XMLDocument* doc)
{
  tinyxml2::XMLPrinter printer;
  doc->Print(&printer);
  return printer.CStr();
}
}  // namespace xml
