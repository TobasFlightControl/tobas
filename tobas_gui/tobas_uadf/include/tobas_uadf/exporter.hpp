#pragma once

#include <tinyxml2.h>

#include "./model.hpp"

namespace uadf
{
tinyxml2::XMLDocument* exportUADF(const Model& model);
}  // namespace uadf
