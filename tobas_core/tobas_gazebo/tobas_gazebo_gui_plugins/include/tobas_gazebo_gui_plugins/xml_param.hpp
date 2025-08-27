#pragma once

#include <tinyxml2.h>

namespace gazebo
{
bool getXmlParam(const tinyxml2::XMLElement* elem, const char* name, double& param);
}  // namespace gazebo
