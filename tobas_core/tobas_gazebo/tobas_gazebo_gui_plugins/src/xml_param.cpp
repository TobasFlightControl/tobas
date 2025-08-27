#include "tobas_gazebo_gui_plugins/xml_param.hpp"

#include <gz/common/Console.hh>

using namespace std;

namespace gazebo
{
bool getXmlParam(const tinyxml2::XMLElement* elem, const char* name, double& param)
{
  const auto child_elem = elem->FirstChildElement(name);
  if (!child_elem) {
    gzerr << "XML parameter \"" << name << "\" does not exist." << endl;
    return false;
  }

  const string text(child_elem->GetText());

  try {
    param = stod(text);
  }
  catch (const exception& e) {
    gzerr << e.what() << endl;
    return false;
  }

  return true;
}
}  // namespace gazebo
