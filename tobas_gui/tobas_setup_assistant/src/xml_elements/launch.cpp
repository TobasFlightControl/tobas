#include <tobas_std_tools/check.hpp>

#include "tobas_setup_assistant/xml_elements/launch.hpp"

using namespace std;

namespace gui
{
namespace sa
{
tinyxml2::XMLElement* addNode(
  tinyxml2::XMLElement* parent,
  const string& pkg,
  const string& exec,
  const string& name,
  const string& ns,
  const string& output,
  const string& args)
{
  const auto node = parent->InsertNewChildElement("node");
  node->SetAttribute("pkg", pkg.c_str());
  node->SetAttribute("exec", exec.c_str());
  if (!name.empty())
    node->SetAttribute("name", name.c_str());
  if (!ns.empty())
    node->SetAttribute("namespace", ns.c_str());
  if (!output.empty())
    node->SetAttribute("output", output.c_str());
  if (!args.empty())
    node->SetAttribute("args", args.c_str());
  return node;
}

tinyxml2::XMLElement* addNodeParam(tinyxml2::XMLElement* node, const string& name, const string& value)
{
  TOBAS_CHECK(strcmp(node->Name(), "node") == 0);

  const auto param = node->InsertNewChildElement("param");
  param->SetAttribute("name", name.c_str());
  param->SetAttribute("value", value.c_str());

  return param;
}
}  // namespace sa
}  // namespace gui
